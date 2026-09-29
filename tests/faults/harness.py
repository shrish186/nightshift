"""Fault-injection harness for sim tests.

- Hypothesis strategies for random fault schedules (which faults, when, in what mix).
- run_with_faults(): steps a sim cell in sim time, injecting faults as they come due.
- CautiousScript: a TEST-ONLY reference sequence (not the controller). It runs one
  load -> machine -> unload cycle, checks every sensor before every move and stops
  everything on any anomaly. Its `breaks` flags switch off individual checks so tests
  can prove the unsafe-event log catches broken logic. In step 4 the real controller
  replaces it under the same harness.
"""

from __future__ import annotations

from collections.abc import Callable, Generator, Iterable
from dataclasses import dataclass
from enum import Enum
from typing import Any

from hypothesis import strategies as st

from cell.config import CellConfig, load_cell_config
from cell.controller.plausibility import IoPlausibilityMonitor
from cell.drivers.robot import RobotStatus
from cell.drivers.sim.cell import SimCell, build_sim_cell
from cell.drivers.sim.cnc import CncFault
from cell.drivers.sim.gripper import GripperFault
from cell.drivers.sim.robot import RobotFault
from cell.drivers.sim.sensors import SensorFault
from tests.conftest import SIM_CELL

DT = 0.1
CYCLE_S = 10.0


class HumanAction(Enum):
    ESTOP = "estop"
    GUARD_OPEN = "guard_open"


Fault = CncFault | RobotFault | GripperFault | SensorFault | HumanAction
ALL_FAULTS: list[Fault] = [*CncFault, *RobotFault, *GripperFault, *SensorFault, *HumanAction]


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
    elif fault is HumanAction.ESTOP:
        cell.safety.press_estop()
    elif fault is HumanAction.GUARD_OPEN:
        cell.safety.open_guard()


def make_cfg(no_release_sensor: bool = False) -> CellConfig:
    """sim-01 with a short machining timeout so hung cycles resolve quickly in tests."""
    raw: dict[str, Any] = load_cell_config(SIM_CELL).model_dump()
    raw["timeouts_s"]["machining"] = CYCLE_S * 3
    if no_release_sensor:
        raw["machine"]["clamp_released_sensor"] = False
        raw["pins"]["unclamped_in"] = None
        raw["unclamp_fallback"] = {
            "release_wait_s": 2.0,
            "pull_pose": "above_fixture",
            "pull_force_limit_n": 40.0,
        }
    return CellConfig.model_validate(raw)


def new_cell(no_release_sensor: bool = False, seed: int = 0) -> SimCell:
    return build_sim_cell(make_cfg(no_release_sensor), seed=seed, cycle_s=CYCLE_S)


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


class _Abort(Exception):
    pass


# Checks a test can switch off to simulate a bug in controller logic.
BREAKS = frozenset(
    {
        "no_plausibility",  # ignore sensor-pair plausibility
        "no_door_open_wait",  # enter the machine without confirming the door is open
        "close_before_retreat",  # close the door before the arm is clear
        "no_release_check",  # pull the part without confirming unclamp / using fallback
        "start_before_retreat",  # cycle start while the arm is still inside
    }
)


class CautiousScript:
    def __init__(self, cell: SimCell, breaks: frozenset[str] = frozenset()) -> None:
        unknown = breaks - BREAKS
        if unknown:
            raise ValueError(f"unknown breaks: {sorted(unknown)}")
        self.cell = cell
        self.breaks = breaks
        self.mon = IoPlausibilityMonitor(cell.cnc, cell.clock, cell.cfg)
        self.stopped = False
        self.stop_reason = ""
        self.done = False
        self._gen = self._sequence()

    # --- driving ---
    def step(self) -> bool:
        """Advance one tick. Returns False once finished or stopped."""
        if self.stopped or self.done:
            return False
        reason = self._anomaly()
        if reason:
            self._stop(reason)
            return False
        try:
            next(self._gen)
        except StopIteration:
            self.done = True
        except _Abort:
            pass
        return not (self.stopped or self.done)

    def _anomaly(self) -> str:
        c = self.cell
        if not c.safety.estop_ok() or not c.safety.guard_closed():
            return "safety input"
        if c.cnc.alarm():
            return "cnc alarm"
        if c.robot.status() in (RobotStatus.FAULT, RobotStatus.STOPPED, RobotStatus.FORCE_LIMIT):
            return f"robot {c.robot.status().value}"
        if "no_plausibility" not in self.breaks:
            faults = self.mon.check()
            if faults:
                return f"plausibility {[f.value for f in faults]}"
        return ""

    def _stop(self, reason: str) -> None:
        self.cell.robot.stop()
        self.cell.cnc.feed_hold()
        self.stopped = True
        self.stop_reason = reason

    # --- helpers ---
    def _wait(self, cond: Callable[[], bool], timeout_s: float, what: str) -> Generator[None]:
        start = self.cell.clock.now()
        while not cond():
            if self.cell.clock.now() - start > timeout_s:
                self._stop(f"timeout: {what}")
                raise _Abort
            yield

    def _move(self, pose: str) -> Generator[None]:
        self.cell.robot.move_to(pose)
        yield from self._wait(
            lambda: self.cell.robot.at_pose() == pose, self.cell.cfg.timeouts_s.load, pose
        )

    def _require(self, ok: bool, what: str) -> None:
        if not ok:
            self._stop(what)
            raise _Abort

    # --- the cycle ---
    def _sequence(self) -> Generator[None]:
        c, t = self.cell, self.cell.cfg.timeouts_s
        # pick raw part
        c.gripper.open()
        yield from self._wait(c.gripper.is_open, 2, "gripper open")
        yield from self._move("above_raw_tray")
        yield from self._move("pick_raw")
        c.gripper.close()
        yield from self._wait(c.gripper.is_closed, 2, "gripper close")
        self._require(c.gripper.has_part(), "no part picked")
        yield from self._move("above_raw_tray")

        # enter and load
        yield from self._open_door()
        yield from self._move("above_fixture")
        yield from self._move("load")
        c.cnc.clamp()
        yield from self._wait(c.cnc.clamped, t.clamp, "clamp")
        c.gripper.open()
        yield from self._wait(c.gripper.is_open, 2, "gripper release")

        # retreat, close, machine
        if "start_before_retreat" in self.breaks:
            yield from self._move("above_fixture")
            c.cnc.cycle_start()
        if "close_before_retreat" not in self.breaks:
            yield from self._move("above_fixture")
            yield from self._move("clear_of_machine")
        c.cnc.close_door()
        yield from self._wait(c.cnc.door_closed, t.door, "door close")
        c.cnc.cycle_start()
        yield from self._wait(c.cnc.cycle_done, t.machining, "cycle")

        # enter and unload
        yield from self._open_door()
        yield from self._move("above_fixture")
        yield from self._move("load")
        c.gripper.close()
        yield from self._wait(c.gripper.is_closed, 2, "gripper close")
        self._require(c.gripper.has_part(), "finished part not gripped")
        c.cnc.unclamp()
        fb = c.cfg.unclamp_fallback
        if "no_release_check" in self.breaks:
            yield from self._move("above_fixture")
        elif fb is None:
            yield from self._wait(c.cnc.unclamped, t.unclamp, "unclamp")
            yield from self._move("above_fixture")
        else:
            start = c.clock.now()
            while c.clock.now() - start < fb.release_wait_s:
                yield
            c.robot.move_to_limited(fb.pull_pose, fb.pull_force_limit_n)
            yield from self._wait(
                lambda: c.robot.at_pose() == fb.pull_pose, t.unload, "fallback pull"
            )

        # leave and place
        yield from self._move("clear_of_machine")
        yield from self._move("above_done_tray")
        yield from self._move("place_done")
        c.gripper.open()
        yield from self._wait(c.gripper.is_open, 2, "gripper release")
        yield from self._move("above_done_tray")
        yield from self._move("home")

    def _open_door(self) -> Generator[None]:
        c = self.cell
        self._require(not c.cnc.cycle_running(), "spindle running")
        c.cnc.open_door()
        if "no_door_open_wait" in self.breaks:
            return
        yield from self._wait(c.cnc.door_open, c.cfg.timeouts_s.door, "door open")
