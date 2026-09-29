"""Run the simulated cell for N cycles, with optional faults and a live terminal view.

    python -m cell.sim.run --cycles 5
    python -m cell.sim.run --cycles 5 --fault TOOL_BREAK@MACHINING+4#3
    python -m cell.sim.run --cycles 8 --random-faults 2 --seed 7 --speed 20
    python -m cell.sim.run --cycles 3 --no-live          # summary only (CI)

Phases:
  1. REFERENCE: if the program has no confirmed watchman reference, a supervised run
     records one. The demo operator confirms a fresh tool, then confirms each cycle clean.
  2. UNATTENDED: the requested cycles, with any faults injected. The run stops at SAFE.

Fault spec: KIND@STATE[+delay_s][#cycle], e.g. DOOR_SENSORS_STUCK_OPEN@OPEN_DOOR_LOAD+1.1
KIND is any sim fault name (see --list-faults); cycle is the 1-based unattended cycle.
Exit code: 0 = no unsafe events (ending in SAFE is a correct outcome), 1 = unsafe events.
"""

from __future__ import annotations

import argparse
import logging
import random
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from cell.config import CellConfig, load_cell_config
from cell.controller.alerts import MemoryAlerter
from cell.controller.machine import CellController
from cell.controller.states import State
from cell.drivers.sensors import SensorFrame, Sensors
from cell.drivers.sim.cell import SimCell, build_sim_cell
from cell.drivers.sim.cnc import CncFault
from cell.drivers.sim.gripper import GripperFault
from cell.drivers.sim.robot import RobotFault
from cell.drivers.sim.safety import SafetyFault
from cell.drivers.sim.sensors import SensorFault
from cell.log import JsonFormatter, get_logger
from cell.watchman.reference import ReferenceStore
from cell.watchman.watchman import ALERT_STATE, Watchman

DT = 0.1
OPERATOR = "demo-operator"
PROGRAM = "DEMO"
TOOL = "T1"
DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "cells" / "sim-01.yaml"


class HumanAction(Enum):
    ESTOP = "estop"
    GUARD_OPEN = "guard_open"


FaultKind = CncFault | RobotFault | GripperFault | SensorFault | SafetyFault | HumanAction
FAULTS: dict[str, FaultKind] = {
    f.name: f
    for enum in (CncFault, RobotFault, GripperFault, SensorFault, SafetyFault, HumanAction)
    for f in enum
}


@dataclass(frozen=True)
class FaultSpec:
    fault: FaultKind
    state: State
    delay_s: float = 0.0
    cycle: int = 1

    def __str__(self) -> str:
        return f"{self.fault.name}@{self.state.name}+{self.delay_s:g}#{self.cycle}"


def parse_fault(spec: str) -> FaultSpec:
    """KIND@STATE[+delay_s][#cycle]"""
    if "@" not in spec:
        raise ValueError(f"fault spec {spec!r}: expected KIND@STATE[+delay][#cycle]")
    kind, rest = spec.split("@", 1)
    cycle = 1
    if "#" in rest:
        rest, c = rest.split("#", 1)
        cycle = int(c)
    delay = 0.0
    if "+" in rest:
        rest, d = rest.split("+", 1)
        try:
            delay = float(d)
        except ValueError as e:
            raise ValueError(f"fault spec {spec!r}: bad delay {d!r}") from e
    fault = FAULTS.get(kind.upper())
    if fault is None:
        raise ValueError(f"fault spec {spec!r}: unknown fault {kind!r} (see --list-faults)")
    try:
        state = State[rest.upper()]
    except KeyError as e:
        raise ValueError(f"fault spec {spec!r}: unknown state {rest!r}") from e
    return FaultSpec(fault, state, delay, cycle)


def inject(cell: SimCell, fault: FaultKind) -> None:
    if isinstance(fault, CncFault):
        cell.cnc.inject(fault)
    elif isinstance(fault, RobotFault):
        cell.robot.inject(fault)
    elif isinstance(fault, GripperFault):
        cell.gripper.inject(fault)
    elif isinstance(fault, SensorFault):
        cell.sensors.inject(fault)
    elif isinstance(fault, SafetyFault):
        cell.safety.inject(fault)
    elif fault is HumanAction.ESTOP:
        cell.safety.press_estop()
    elif fault is HumanAction.GUARD_OPEN:
        cell.safety.open_guard()


def demo_config(path: Path, cycle_s: float, no_release_sensor: bool) -> CellConfig:
    """The cell config with one demo program whose expected cycle matches the sim."""
    raw: dict[str, Any] = load_cell_config(path).model_dump()
    raw["programs"] = {PROGRAM: {"expected_cycle_s": cycle_s, "tool": TOOL}}
    if no_release_sensor:
        raw["machine"]["clamp_released_sensor"] = False
        raw["pins"]["unclamped_in"] = None
        raw["unclamp_fallback"] = {
            "release_wait_s": 2.0,
            "pull_pose": "above_fixture",
            "pull_force_limit_n": 40.0,
            "tug_pose": "tug_in_fixture",
            "tug_force_n": 20.0,
        }
    return CellConfig.model_validate(raw)


class RecordingSensors:
    """Passes frames through to the watchman and keeps recent ones for the display."""

    def __init__(self, inner: Sensors, keep: int = 80) -> None:
        self._inner = inner
        self.frames: deque[SensorFrame] = deque(maxlen=keep)

    def read(self) -> SensorFrame | None:
        frame = self._inner.read()
        if frame is not None and (not self.frames or frame.ts > self.frames[-1].ts):
            self.frames.append(frame)
        return frame


@dataclass
class Run:
    cfg: CellConfig
    cell: SimCell
    ctrl: CellController
    watchman: Watchman
    alerter: MemoryAlerter
    sensors: RecordingSensors
    speed: float
    phase: str = ""
    cycle_label: str = ""
    faults_done: list[str] = field(default_factory=list)
    events: deque[str] = field(default_factory=lambda: deque(maxlen=9))
    reference_cycles_recorded: int = 0
    unattended_done: int = 0
    _seen_transitions: int = 0
    _seen_alerts: int = 0

    def note(self, text: str) -> None:
        self.events.append(f"{self.cell.clock.now():7.1f}s  {text}")

    def collect_events(self) -> None:
        for t in self.ctrl.history[self._seen_transitions :]:
            self.note(f"{t.src.value} -> {t.dst.value}  ({t.reason})")
        self._seen_transitions = len(self.ctrl.history)
        for a in self.alerter.alerts[self._seen_alerts :]:
            tag = "ALERT" if a.state == ALERT_STATE else "SAFE ALERT"
            self.note(f"{tag}: {a.reason}")
        self._seen_alerts = len(self.alerter.alerts)


def build(args: argparse.Namespace) -> Run:
    cfg = demo_config(args.config, args.cycle_s, args.no_release_sensor)
    cell = build_sim_cell(cfg, seed=args.seed, cycle_s=args.cycle_s)
    alerter = MemoryAlerter()
    ctrl = CellController(cfg, cell.clock, cell.robot, cell.gripper, cell.cnc, cell.safety, alerter)
    sensors = RecordingSensors(cell.sensors)
    store = ReferenceStore(args.store)
    watchman = Watchman(cfg, cell.clock, sensors, cell.cnc, ctrl.request_safe, alerter, store)
    ctrl.reset_checks.append(watchman.health)
    ctrl.start_checks.append(watchman.prepare)
    return Run(cfg, cell, ctrl, watchman, alerter, sensors, args.speed)


def run_cycle(run: Run, supervised: bool, faults: list[FaultSpec], live: Live | None) -> bool:
    """One cycle. True if it completed back in IDLE."""
    ctrl, cell = run.ctrl, run.cell
    if not ctrl.start_cycle(PROGRAM, supervised=supervised):
        run.note(f"start refused: {ctrl.last_start_refusal}")
        return False
    start = ctrl.cycles_completed
    entered: dict[FaultSpec, float] = {}
    for _ in range(int(3 * run.cfg.programs[PROGRAM].expected_cycle_s / DT) + 3000):
        for spec in faults:
            if ctrl.state is spec.state and spec not in entered:
                entered[spec] = cell.clock.now()
            if spec in entered and cell.clock.now() - entered[spec] >= spec.delay_s - 1e-9:
                inject(cell, spec.fault)
                run.faults_done.append(str(spec))
                run.note(f"FAULT INJECTED: {spec.fault.name}")
                faults.remove(spec)
                break
        run.watchman.tick()
        ctrl.step()
        run.collect_events()
        if live is not None:
            live.update(render(run))
            time.sleep(DT / run.speed)
        if ctrl.state is State.SAFE or (ctrl.state is State.IDLE and ctrl.cycles_completed > start):
            break
        cell.clock.advance(DT)
    return ctrl.state is State.IDLE and ctrl.cycles_completed > start


def execute(run: Run, args: argparse.Namespace, live: Live | None) -> None:
    w = run.watchman
    if w.reference_for(PROGRAM) is None:
        run.phase = "REFERENCE (supervised)"
        w.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=True)
        run.note(f"operator {OPERATOR} confirms a fresh tool {TOOL}")
        while w.recording_progress is not None:
            n, total = w.recording_progress
            run.cycle_label = f"reference cycle {n + 1}/{total}"
            if not run_cycle(run, supervised=True, faults=[], live=live):
                return
            why = w.confirm_cycle_clean(OPERATOR)
            run.note(f"operator confirms cycle clean -> {why or 'accepted'}")
            run.reference_cycles_recorded += 1

    random_faults = []
    if args.random_faults:
        rng = random.Random(args.seed)
        working = [s for s in State if s not in (State.IDLE, State.SAFE)]
        kinds = list(FAULTS.values())
        for _ in range(args.random_faults):
            random_faults.append(
                FaultSpec(
                    rng.choice(kinds),
                    rng.choice(working),
                    round(rng.uniform(0, 3), 1),
                    rng.randint(1, args.cycles),
                )
            )
    pending = list(args.fault) + random_faults
    run.phase = "UNATTENDED"
    for i in range(1, args.cycles + 1):
        run.cycle_label = f"cycle {i}/{args.cycles}"
        now_faults = [f for f in pending if f.cycle == i]
        if not run_cycle(run, supervised=False, faults=now_faults, live=live):
            return
        run.unattended_done += 1


# --- live view ---

_BLOCKS = " ▁▂▃▄▅▆▇█"
_STATE_STYLE = {
    State.SAFE: "bold white on red",
    State.IDLE: "bold green",
    State.MACHINING: "bold cyan",
}


def spark(values: list[float], top: float) -> str:
    return "".join(_BLOCKS[min(8, max(0, round(8 * v / top)))] for v in values)


def _yn(v: bool, good: bool = True) -> Text:
    return Text("ON " if v else "off", style=("green" if v == good else "yellow"))


def render(run: Run) -> Layout:
    c, ctrl, w, cfg = run.cell, run.ctrl, run.watchman, run.cfg
    header = Text.assemble(
        ("NightShift sim  ", "bold"),
        f"cell {cfg.cell_id}  program {PROGRAM}/{TOOL}  ",
        (f"{run.phase}  ", "bold magenta"),
        f"{run.cycle_label}  t={c.clock.now():.1f}s  speed {run.speed:g}x",
    )

    state = Text(f" {ctrl.state.value} ", style=_STATE_STYLE.get(ctrl.state, "bold yellow"))
    last = ctrl.history[-1].reason if ctrl.history else "-"
    ctl = Table.grid(padding=(0, 1))
    ctl.add_row("state", state)
    ctl.add_row("last", Text(last, overflow="fold"))
    ctl.add_row("cycles", str(ctrl.cycles_completed))

    m = Table("", "sensor", "physical", box=None, padding=(0, 1))
    m.add_row("door open", _yn(c.cnc.door_open()), c.cnc.door())
    m.add_row("door closed", _yn(c.cnc.door_closed()), "")
    m.add_row("clamped", _yn(c.cnc.clamped()), f"jaws {c.cnc.jaw_position():.2f}")
    m.add_row("released", _yn(c.cnc.unclamped()), "")
    m.add_row("part seated", _yn(c.cnc.part_present()), c.cnc.seat())
    m.add_row("spindle", _yn(c.cnc.cycle_running()), "cutting" if c.cnc.spindle_cutting() else "")
    m.add_row("feed hold", _yn(c.cnc.feed_hold_active(), good=False), "")
    m.add_row("alarm", _yn(c.cnc.alarm(), good=False), "")

    r = Table.grid(padding=(0, 1))
    r.add_row("pose", c.robot.at_pose() or "moving...")
    r.add_row("status", c.robot.status().value)
    grip = "closed" if c.gripper.is_closed() else "open" if c.gripper.is_open() else "moving"
    r.add_row("gripper", f"{grip}{' + part' if c.gripper.has_part() else ''}")
    r.add_row("interlock", f"{c.interlock_blocks} blocked" if c.interlock_blocks else "ok")

    frames = list(run.sensors.frames)
    wc = cfg.watchman
    amps = [f.spindle_current_a for f in frames]
    vib = [f.vibration_rms_g for f in frames]
    ref = w.reference_for(PROGRAM)
    wt = Table.grid(padding=(0, 1))
    wt.add_row(
        "current",
        Text(spark(amps, wc.overload_current_a), style="cyan"),
        f"{amps[-1]:5.1f} A" if amps else "",
    )
    wt.add_row(
        "vibration",
        Text(spark(vib, wc.vibration_rms_max_g), style="magenta"),
        f"{vib[-1]:5.2f} g" if vib else "",
    )
    if ref is not None:
        lim = ref.limits(wc)
        wt.add_row(
            "reference",
            f"{ref.level_mean:.1f} A by {ref.operator}",
            f"alert wear x{lim.wear_alert:.2f} chip x{lim.chip_alert:.2f}",
        )
    elif w.recording_progress is not None:
        n, total = w.recording_progress
        wt.add_row("reference", f"recording {n}/{total}", "")
    wt.add_row(
        "stops at",
        f"overload {wc.overload_current_a:g} A / {wc.vibration_rms_max_g:g} g",
        f"break < x{wc.tool_break_current_ratio:g}",
    )
    wt.add_row(
        "status",
        Text("STOPPED: " + w.stop_reason, style="red")
        if w.stopped
        else Text("watching", style="green"),
        "",
    )

    unsafe = len(c.violations)
    footer = Text.assemble(
        ("unsafe events: ", "bold"),
        (str(unsafe), "bold green" if unsafe == 0 else "bold white on red"),
        f"   faults injected: {', '.join(run.faults_done) or 'none'}",
    )

    layout = Layout()
    layout.split_column(
        Layout(Panel(header), size=3),
        Layout(name="mid", size=12),
        Layout(Panel(wt, title="Watchman"), size=8),
        Layout(Panel("\n".join(run.events), title="Events"), size=11),
        Layout(Panel(footer), size=3),
    )
    layout["mid"].split_row(
        Layout(Panel(ctl, title="Controller")),
        Layout(Panel(m, title="Machine")),
        Layout(Panel(r, title="Robot")),
    )
    return layout


def summary(run: Run, args: argparse.Namespace) -> str:
    ref = run.watchman.reference_for(PROGRAM)
    lines = [
        f"NightShift sim run: cell {run.cfg.cell_id}, program {PROGRAM} (tool {TOOL}), "
        f"seed {args.seed}"
    ]
    if ref is not None:
        lim = ref.limits(run.cfg.watchman)
        how = (
            f"recorded {run.reference_cycles_recorded} supervised cycles (operator-confirmed)"
            if run.reference_cycles_recorded
            else "loaded from store"
        )
        lines.append(
            f"reference: {how}; alert limits wear x{lim.wear_alert:.2f} chip x{lim.chip_alert:.2f}"
        )
    lines.append(f"unattended cycles: {run.unattended_done}/{args.cycles} completed")
    final = run.ctrl.state.value
    if run.ctrl.state is State.SAFE:
        final += f" ({run.ctrl.last_safe_reason})"
    lines.append(f"final state: {final}")
    lines.append(f"faults injected: {', '.join(run.faults_done) or 'none'}")
    alerts = [a.reason for a in run.alerter.alerts if a.state == ALERT_STATE]
    lines.append(f"watchman alerts: {len(alerts)}" + (f" ({'; '.join(alerts)})" if alerts else ""))
    lines.append(f"interlock blocks: {run.cell.interlock_blocks}")
    lines.append(f"unsafe events: {len(run.cell.violations)}")
    lines += [f"  UNSAFE: {v}" for v in run.cell.violations]
    return "\n".join(lines)


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="python -m cell.sim.run",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--cycles", type=int, default=5, help="unattended cycles (default 5)")
    p.add_argument("--cycle-s", type=float, default=12.0, help="machining time per cycle")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    p.add_argument(
        "--no-release-sensor",
        action="store_true",
        help="machine without a clamp-released sensor (tug test + fallback pull)",
    )
    p.add_argument(
        "--fault", type=parse_fault, action="append", default=[], metavar="KIND@STATE[+s][#cycle]"
    )
    p.add_argument("--random-faults", type=int, default=0, metavar="K")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--speed", type=float, default=10.0, help="sim seconds per real second")
    p.add_argument(
        "--store",
        type=Path,
        default=None,
        help="reference store JSON (default: in memory, recorded each run)",
    )
    p.add_argument("--no-live", action="store_true", help="no live view; print a summary")
    p.add_argument("--log", type=Path, default=None, help="append JSON event log here")
    p.add_argument("--list-faults", action="store_true")
    return p.parse_args(argv)


_LOGGERS = ("cell.controller", "cell.watchman")


def route_logs(path: Path | None) -> list[tuple[logging.Logger, list[logging.Handler], bool]]:
    """Keep the JSON event logs off the console (the view/summary replaces them); write
    them to `path` if given. Returns the previous setup so main() can restore it."""
    saved = []
    for name in _LOGGERS:
        logger = get_logger(name)
        saved.append((logger, list(logger.handlers), logger.propagate))
        for h in list(logger.handlers):
            logger.removeHandler(h)
        logger.propagate = False
        if path is None:
            logger.addHandler(logging.NullHandler())
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        handler = logging.FileHandler(path, mode="a")
        handler.setFormatter(JsonFormatter())
        logger.addHandler(handler)
    return saved


def restore_logs(saved: list[tuple[logging.Logger, list[logging.Handler], bool]]) -> None:
    for logger, handlers, propagate in saved:
        for h in list(logger.handlers):
            logger.removeHandler(h)
            h.close()
        for h in handlers:
            logger.addHandler(h)
        logger.propagate = propagate


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.list_faults:
        print("\n".join(sorted(FAULTS)))
        return 0
    saved = route_logs(args.log)
    try:
        run = build(args)
        if args.no_live:
            execute(run, args, None)
        else:
            console = Console()
            with Live(render(run), console=console, refresh_per_second=15) as live:
                execute(run, args, live)
                live.update(render(run))
                time.sleep(min(3.0, 30 / run.speed))
            console.print(Group(Text()))
        print(summary(run, args))
        return 0 if not run.cell.violations else 1
    finally:
        restore_logs(saved)


if __name__ == "__main__":
    sys.exit(main())
