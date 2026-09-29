"""One test per fault: the controller must end in SAFE with the cell stopped, an
alert raised, the transition logged, and no unsafe event."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import pytest

from cell.controller.alerts import MemoryAlerter
from cell.controller.machine import CellController
from cell.controller.states import State
from cell.drivers.robot import RobotStatus
from cell.drivers.sim.cell import SimCell
from cell.drivers.sim.cnc import CncFault
from cell.drivers.sim.gripper import GripperFault
from cell.drivers.sim.robot import RobotFault
from tests.faults.harness import new_cell

S = State
STOPPED = (RobotStatus.STOPPED, RobotStatus.FAULT, RobotStatus.FORCE_LIMIT)


def make(cell: SimCell) -> tuple[CellController, MemoryAlerter]:
    alerter = MemoryAlerter()
    ctrl = CellController(
        cell.cfg, cell.clock, cell.robot, cell.gripper, cell.cnc, cell.safety, alerter
    )
    return ctrl, alerter


def run_until(
    cell: SimCell, ctrl: CellController, done: Callable[[], bool], max_s: float = 200.0
) -> None:
    t0 = cell.clock.now()
    while cell.clock.now() - t0 < max_s:
        ctrl.step()
        if done() or ctrl.state is S.SAFE:
            return
        cell.clock.advance(0.1)


@dataclass(frozen=True)
class Case:
    at: State
    act: Callable[[SimCell, CellController], None]
    expect: str
    no_release_sensor: bool = False


def cnc(f: CncFault) -> Callable[[SimCell, CellController], None]:
    return lambda cell, ctrl: cell.cnc.inject(f)


def robot(f: RobotFault) -> Callable[[SimCell, CellController], None]:
    return lambda cell, ctrl: cell.robot.inject(f)


def gripper(f: GripperFault) -> Callable[[SimCell, CellController], None]:
    return lambda cell, ctrl: cell.gripper.inject(f)


CASES = {
    "estop-mid-move": Case(S.PICK_RAW, lambda c, _: c.safety.press_estop(), "e-stop"),
    "guard-opened": Case(S.LOAD, lambda c, _: c.safety.open_guard(), "guard open"),
    "door-wont-open": Case(S.PICK_RAW, cnc(CncFault.DOOR_STUCK), "door_both_off_too_long"),
    "door-wont-close": Case(S.RETREAT, cnc(CncFault.DOOR_STUCK), "door_both_off_too_long"),
    "clamp-timeout": Case(S.LOAD, cnc(CncFault.CLAMP_FAIL), "CLAMP timeout"),
    "empty-grip-at-pick": Case(S.PICK_RAW, gripper(GripperFault.EMPTY_GRIP), "raw part"),
    "empty-grip-at-unload": Case(
        S.MACHINING, gripper(GripperFault.EMPTY_GRIP), "finished part in gripper"
    ),
    "part-dropped": Case(S.OPEN_DOOR_LOAD, gripper(GripperFault.DROP), "raw part in gripper"),
    "robot-fault": Case(S.LOAD, robot(RobotFault.FAULT), "robot fault"),
    "robot-stall": Case(S.LOAD, robot(RobotFault.STALL), "LOAD timeout"),
    "cnc-alarm": Case(S.MACHINING, cnc(CncFault.ALARM), "CNC alarm"),
    "cycle-hang": Case(S.CLOSE_DOOR, cnc(CncFault.CYCLE_HANG), "MACHINING timeout"),
    "door-sensor-short": Case(S.MACHINING, cnc(CncFault.DOOR_SENSOR_SHORT), "door_both_on"),
    "clamp-sensor-short": Case(S.MACHINING, cnc(CncFault.CLAMP_SENSOR_SHORT), "clamp_both_on"),
    "io-power-loss": Case(S.RETREAT, cnc(CncFault.IO_POWER_LOSS), "CNC alarm"),
    "clamp-stuck-on-with-sensor": Case(
        S.MACHINING, cnc(CncFault.CLAMP_STUCK_ON), "UNCLAMP timeout"
    ),
    "clamp-stuck-on-no-sensor": Case(
        S.MACHINING, cnc(CncFault.CLAMP_STUCK_ON), "robot force_limit", no_release_sensor=True
    ),
    "door-closes-while-arm-inside": Case(
        S.LOAD, cnc(CncFault.DOOR_CLOSES_UNCOMMANDED), "guard arm_may_be_inside"
    ),
    "part-misseated": Case(
        S.PICK_RAW, cnc(CncFault.PART_MISSEATED), "CLAMP timeout: part seated in fixture"
    ),
    "part-dropped-before-clamp": Case(
        S.CLAMP, gripper(GripperFault.DROP), "CLAMP timeout: part seated in fixture"
    ),
    "part-knocked-crooked-while-cutting": Case(
        S.MACHINING, cnc(CncFault.PART_MISSEATED), "guard machine_ready_to_cut: part not seated"
    ),
    "gripper-closed-before-unload": Case(
        S.OPEN_DOOR_UNLOAD,
        lambda c, _: c.gripper.close(),
        "ENTER_UNLOAD requirement failed: gripper open",
    ),
    "feed-hold-pressed-mid-cut": Case(
        S.MACHINING, lambda c, _: c.cnc.feed_hold(), "feed hold during machining"
    ),
    "watchman-request": Case(
        S.MACHINING, lambda _, ctrl: ctrl.request_safe("watchman: tool break"), "watchman"
    ),
}


@pytest.mark.parametrize("case", CASES.values(), ids=CASES.keys())
def test_fault_goes_safe(case: Case) -> None:
    cell = new_cell(case.no_release_sensor)
    ctrl, alerter = make(cell)
    assert ctrl.start_cycle()
    run_until(cell, ctrl, lambda: ctrl.state is case.at)
    assert ctrl.state is case.at, ctrl.last_safe_reason
    case.act(cell, ctrl)
    run_until(cell, ctrl, lambda: ctrl.state is S.IDLE)

    assert ctrl.state is S.SAFE
    assert case.expect in ctrl.last_safe_reason, ctrl.last_safe_reason
    assert cell.cnc.feed_hold_active()
    assert cell.robot.status() in STOPPED
    assert len(alerter.alerts) == 1 and case.expect in alerter.alerts[0].reason
    assert ctrl.history[-1].dst is S.SAFE
    assert cell.violations == []


def test_every_fault_type_is_covered() -> None:
    """New sim faults must get a controller test (sensor faults are the watchman's)."""
    covered = {"DOOR_STUCK", "CLAMP_FAIL", "ALARM", "CYCLE_HANG", "CLAMP_STUCK_ON",
               "DOOR_SENSOR_SHORT", "CLAMP_SENSOR_SHORT", "IO_POWER_LOSS", "FAULT", "STALL",
               "EMPTY_GRIP", "DROP", "DOOR_CLOSES_UNCOMMANDED", "PART_MISSEATED"}  # fmt: skip
    all_faults = {f.name for f in (*CncFault, *RobotFault, *GripperFault)}
    # CLAMP_JAM and gripper STUCK are exercised by the random fault tests.
    assert all_faults - covered == {"CLAMP_JAM", "STUCK"}
