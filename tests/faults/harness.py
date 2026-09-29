"""Fault-injection harness for sim tests.

- Hypothesis strategies for random fault schedules (which faults, when, in what mix).
- run_with_faults(): steps a sim cell in sim time, injecting faults as they come due.
- ControllerRunner: drives the real CellController one cycle at a time, for
  run_cycles() (false stops) and run_with_faults() (random faults).
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Protocol

from hypothesis import strategies as st

from cell.config import CellConfig, load_cell_config
from cell.controller.alerts import MemoryAlerter
from cell.controller.machine import CellController
from cell.controller.states import State
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


class ControllerRunner:
    """One cycle of a CellController. done = back in IDLE with one more cycle; stopped = SAFE."""

    def __init__(self, ctrl: CellController) -> None:
        self.ctrl = ctrl
        self._start = ctrl.cycles_completed
        ctrl.start_cycle()

    @property
    def done(self) -> bool:
        return self.ctrl.state is State.IDLE and self.ctrl.cycles_completed > self._start

    @property
    def stopped(self) -> bool:
        return self.ctrl.state is State.SAFE

    @property
    def stop_reason(self) -> str:
        return self.ctrl.last_safe_reason

    def step(self) -> bool:
        self.ctrl.step()
        return not (self.done or self.stopped)


def controller_runners() -> Callable[[SimCell], Runner]:
    """Runner factory for run_cycles(): one controller per cell, kept across cycles."""
    ctrls: dict[int, CellController] = {}

    def new(cell: SimCell) -> Runner:
        if id(cell) not in ctrls:
            ctrls[id(cell)] = make_controller(cell)[0]
        return ControllerRunner(ctrls[id(cell)])

    return new


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
    cfg = make_cfg(v.no_release_sensor)
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
