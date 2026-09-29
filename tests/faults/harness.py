"""Fault-injection harness for sim tests.

- Hypothesis strategies for random fault schedules (which faults, when, in what mix).
- run_with_faults(): steps a sim cell in sim time, injecting faults as they come due.
- make_system(): the real CellController + Watchman sharing one alerter.
- ControllerRunner: drives them one cycle at a time, for run_cycles() (false stops)
  and run_with_faults() (random faults).
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol

from hypothesis import strategies as st

from cell.config import CellConfig, load_cell_config
from cell.controller.alerts import Alert, MemoryAlerter
from cell.controller.machine import CellController
from cell.controller.states import State
from cell.drivers.sim.cell import SimCell, build_sim_cell
from cell.drivers.sim.cnc import CncFault
from cell.drivers.sim.gripper import GripperFault
from cell.drivers.sim.robot import RobotFault
from cell.drivers.sim.safety import SafetyFault
from cell.drivers.sim.sensors import SensorFault
from cell.watchman.reference import Reference, ReferenceStore
from cell.watchman.watchman import ALERT_STATE, Watchman
from tests.conftest import SIM_CELL

DT = 0.1
CYCLE_S = 10.0
PROGRAM = "P1"  # the test program; expected_cycle_s matches the sim's cycle length
TOOL = "T1"


class HumanAction(Enum):
    ESTOP = "estop"
    GUARD_OPEN = "guard_open"


Fault = CncFault | RobotFault | GripperFault | SensorFault | SafetyFault | HumanAction
ALL_FAULTS: list[Fault] = [
    *CncFault,
    *RobotFault,
    *GripperFault,
    *SensorFault,
    *SafetyFault,
    *HumanAction,
]


@dataclass(frozen=True)
class FaultEvent:
    at_s: float
    fault: Fault


def fault_schedules(
    max_s: float = 120.0, max_faults: int = 3
) -> st.SearchStrategy[list[FaultEvent]]:
    event = st.builds(
        FaultEvent,
        at_s=st.floats(min_value=0.0, max_value=max_s, allow_nan=False).map(lambda t: round(t, 1)),
        fault=st.sampled_from(ALL_FAULTS),
    )
    return st.lists(event, min_size=0, max_size=max_faults)


def inject(cell: SimCell, fault: Fault) -> None:
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


def make_cfg(no_release_sensor: bool = False, cycle_s: float = CYCLE_S) -> CellConfig:
    """sim-01 with one test program whose expected cycle matches the sim's cycle length."""
    raw: dict[str, Any] = load_cell_config(SIM_CELL).model_dump()
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


def new_cell(
    no_release_sensor: bool = False,
    seed: int = 0,
    cycle_s: float = CYCLE_S,
    zone_interlock: bool | None = None,
) -> SimCell:
    """zone_interlock=False: the software alone, without the hardware door-zone interlock."""
    return build_sim_cell(
        make_cfg(no_release_sensor, cycle_s),
        seed=seed,
        cycle_s=cycle_s,
        zone_interlock=zone_interlock,
    )


class Runner(Protocol):
    """What the harness drives for one cycle."""

    @property
    def done(self) -> bool: ...
    @property
    def stopped(self) -> bool: ...
    @property
    def stop_reason(self) -> str: ...

    def step(self) -> bool: ...


def make_controller(cell: SimCell) -> tuple[CellController, MemoryAlerter]:
    alerter = MemoryAlerter()
    ctrl = CellController(
        cell.cfg, cell.clock, cell.robot, cell.gripper, cell.cnc, cell.safety, alerter
    )
    return ctrl, alerter


@dataclass
class System:
    """Controller + watchman sharing one alerter, as they run on a real cell."""

    ctrl: CellController
    watchman: Watchman
    alerter: MemoryAlerter

    @property
    def watchman_alerts(self) -> list[Alert]:
        return [a for a in self.alerter.alerts if a.state == ALERT_STATE]

    @property
    def safe_alerts(self) -> list[Alert]:
        return [a for a in self.alerter.alerts if a.state != ALERT_STATE]


OPERATOR = "test-operator"
_REFERENCES: dict[tuple[bool, float], Reference] = {}


def recorded_reference(no_release_sensor: bool, cycle_s: float) -> Reference:
    """A reference recorded the real way (supervised run, fresh tool confirmed, each
    cycle confirmed clean) on a fresh sim cell. Recorded once per variant and cached."""
    key = (no_release_sensor, round(cycle_s, 3))
    if key not in _REFERENCES:
        cell = new_cell(no_release_sensor, seed=4242, cycle_s=cycle_s)
        system = make_system(cell, with_reference=False)
        w = system.watchman
        assert w.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=True) is None
        while w.recording_progress is not None:
            runner = ControllerRunner(system.ctrl, w, supervised=True)
            run_with_faults(cell, runner.step)
            assert runner.done, system.ctrl.last_safe_reason
            assert w.confirm_cycle_clean(OPERATOR) is None
        ref = w.reference_for(PROGRAM)
        assert ref is not None
        _REFERENCES[key] = ref
    return _REFERENCES[key]


def make_system(cell: SimCell, with_reference: bool = True) -> System:
    """Controller + watchman. with_reference preloads a supervised-recorded reference for
    the test program, so unattended cycles are allowed."""
    ctrl, alerter = make_controller(cell)
    store = ReferenceStore(None)
    if with_reference:
        no_release = not cell.cfg.machine.clamp_released_sensor
        store.put(recorded_reference(no_release, cell.cfg.programs[PROGRAM].expected_cycle_s))
    watchman = Watchman(
        cell.cfg, cell.clock, cell.sensors, cell.cnc, ctrl.request_safe, alerter, store
    )
    ctrl.reset_checks.append(watchman.health)
    ctrl.start_checks.append(watchman.prepare)
    return System(ctrl, watchman, alerter)


def stop_source(reason: str) -> str:
    """Who stopped the cell, for the false-stop report."""
    return "Watchman" if reason.startswith("requested: watchman") else "CellController"


class ControllerRunner:
    """One cycle of a CellController (and its watchman, ticked first each step).

    done = back in IDLE with one more cycle; stopped = SAFE.
    """

    def __init__(
        self,
        ctrl: CellController,
        watchman: Watchman | None = None,
        program: str = PROGRAM,
        supervised: bool = False,
    ) -> None:
        self.ctrl = ctrl
        self.watchman = watchman
        self._start = ctrl.cycles_completed
        self.started = ctrl.start_cycle(program, supervised)

    @property
    def done(self) -> bool:
        return self.ctrl.state is State.IDLE and self.ctrl.cycles_completed > self._start

    @property
    def stopped(self) -> bool:
        return self.ctrl.state is State.SAFE or not self.started

    @property
    def stop_reason(self) -> str:
        return self.ctrl.last_safe_reason if self.started else self.ctrl.last_start_refusal

    def step(self) -> bool:
        if self.watchman is not None:
            self.watchman.tick()
        self.ctrl.step()
        return not (self.done or self.stopped)


def system_runners() -> tuple[Callable[[SimCell], Runner], dict[int, System]]:
    """Runner factory for run_cycles(): one controller + watchman per cell, kept across
    cycles. Also returns the systems so callers can inspect alerts."""
    systems: dict[int, System] = {}

    def new(cell: SimCell) -> Runner:
        if id(cell) not in systems:
            systems[id(cell)] = make_system(cell)
        s = systems[id(cell)]
        return ControllerRunner(s.ctrl, s.watchman)

    return new, systems


@dataclass(frozen=True)
class NoFaultVariation:
    """Everything that varies between runs when the fault injector is disabled."""

    seed: int
    no_release_sensor: bool
    door_scale: float  # real stroke time vs. the configured measured travel
    clamp_scale: float
    robot_speed_scale: float
    cycle_s: float


def no_fault_variations() -> st.SearchStrategy[NoFaultVariation]:
    # Stroke times stay within measured travel +/- a little, always inside the
    # plausibility window (a slower real machine means the config was measured wrong).
    return st.builds(
        NoFaultVariation,
        seed=st.integers(min_value=0, max_value=2**16),
        no_release_sensor=st.booleans(),
        door_scale=st.floats(min_value=0.8, max_value=1.1),
        clamp_scale=st.floats(min_value=0.8, max_value=1.1),
        robot_speed_scale=st.floats(min_value=0.8, max_value=1.2),
        cycle_s=st.floats(min_value=5.0, max_value=20.0),
    )


def cell_for(v: NoFaultVariation) -> SimCell:
    cfg = make_cfg(v.no_release_sensor, v.cycle_s)
    p = cfg.io_plausibility
    return build_sim_cell(
        cfg,
        seed=v.seed,
        cycle_s=v.cycle_s,
        door_s=p.door_travel_s * v.door_scale,
        clamp_s=p.clamp_travel_s * v.clamp_scale,
        robot_speed_scale=v.robot_speed_scale,
    )


@dataclass
class CycleResult:
    completed: int = 0
    stops: list[str] = field(default_factory=list)


def run_cycles(
    cell: SimCell, new_runner: Callable[[SimCell], Runner], n_cycles: int
) -> CycleResult:
    """Run n back-to-back cycles with no faults injected. Stops at the first stop."""
    result = CycleResult()
    for _ in range(n_cycles):
        runner = new_runner(cell)
        run_with_faults(cell, runner.step)
        if not runner.done:
            result.stops.append(runner.stop_reason or "did not finish")
            break
        result.completed += 1
    return result


# Session-wide false-stop tally, printed by tests/conftest.py at the end of pytest.
FALSE_STOP_REPORT: dict[str, dict[str, Any]] = {}


def run_with_faults(
    cell: SimCell,
    step: Callable[[], bool],
    schedule: Iterable[FaultEvent] = (),
    max_s: float = 200.0,
) -> None:
    """Step until step() returns False (finished/stopped) or max_s of sim time passes."""
    pending = sorted(schedule, key=lambda e: e.at_s)
    t0 = cell.clock.now()
    while cell.clock.now() - t0 < max_s:
        while pending and pending[0].at_s <= cell.clock.now() - t0:
            inject(cell, pending.pop(0).fault)
        if not step():
            return
        cell.clock.advance(DT)
