from __future__ import annotations

import pytest

from cell.config import CellConfig, load_cell_config
from cell.drivers.cnc_io import CncIo
from cell.drivers.gripper import Gripper
from cell.drivers.robot import Robot, RobotStatus
from cell.drivers.safety_in import SafetyInputs
from cell.drivers.sensors import Sensors
from cell.drivers.sim.cell import SimCell, build_sim_cell
from cell.drivers.sim.cnc import CncFault
from cell.drivers.sim.gripper import GripperFault
from cell.drivers.sim.robot import RobotFault
from cell.drivers.sim.sensors import CUT_CURRENT_A, SensorFault
from tests.conftest import SIM_CELL


@pytest.fixture
def cell() -> SimCell:
    return build_sim_cell(load_cell_config(SIM_CELL), cycle_s=10.0)


def test_sims_satisfy_interfaces(cell: SimCell) -> None:
    # Checked by mypy: sims must be usable wherever the protocols are expected.
    robot: Robot = cell.robot
    gripper: Gripper = cell.gripper
    cnc: CncIo = cell.cnc
    safety: SafetyInputs = cell.safety
    sensors: Sensors = cell.sensors
    assert all(x is not None for x in (robot, gripper, cnc, safety, sensors))


# --- robot ---


def test_robot_moves_and_arrives(cell: SimCell) -> None:
    cell.robot.move_to("above_raw_tray")
    assert cell.robot.status() is RobotStatus.MOVING
    assert cell.robot.at_pose() is None
    cell.clock.advance(10)
    assert cell.robot.status() is RobotStatus.IDLE
    assert cell.robot.at_pose() == "above_raw_tray"


def test_robot_rejects_unknown_pose(cell: SimCell) -> None:
    with pytest.raises(ValueError):
        cell.robot.move_to("nowhere")


def test_robot_stop_latches_until_reset(cell: SimCell) -> None:
    cell.robot.move_to("above_raw_tray")
    cell.robot.stop()
    assert cell.robot.status() is RobotStatus.STOPPED
    cell.robot.move_to("home")  # ignored while stopped
    assert cell.robot.status() is RobotStatus.STOPPED
    cell.robot.reset()
    assert cell.robot.status() is RobotStatus.IDLE


def test_robot_fault_and_stall(cell: SimCell) -> None:
    cell.robot.move_to("above_raw_tray")
    cell.robot.inject(RobotFault.STALL)
    cell.clock.advance(100)
    assert cell.robot.status() is RobotStatus.MOVING
    cell.robot.inject(RobotFault.FAULT)
    assert cell.robot.status() is RobotStatus.FAULT


# --- gripper ---


def test_gripper_picks_only_where_a_part_is(cell: SimCell) -> None:
    cell.gripper.close()
    cell.clock.advance(1)
    assert cell.gripper.is_closed() and not cell.gripper.has_part()  # at home: nothing there
    cell.gripper.open()
    cell.robot.move_to("above_raw_tray")
    cell.clock.advance(10)
    cell.robot.move_to("pick_raw")
    cell.clock.advance(10)
    cell.gripper.close()
    assert not cell.gripper.is_closed()  # still actuating
    cell.clock.advance(1)
    assert cell.gripper.has_part()


@pytest.mark.parametrize("fault", [GripperFault.EMPTY_GRIP, GripperFault.DROP])
def test_gripper_loses_part(cell: SimCell, fault: GripperFault) -> None:
    cell.robot.move_to("pick_raw")
    cell.clock.advance(10)
    if fault is GripperFault.EMPTY_GRIP:
        cell.gripper.inject(fault)
    cell.gripper.close()
    cell.clock.advance(1)
    if fault is GripperFault.DROP:
        assert cell.gripper.has_part()
        cell.gripper.inject(fault)
    assert not cell.gripper.has_part()


def test_gripper_stuck(cell: SimCell) -> None:
    cell.gripper.inject(GripperFault.STUCK)
    cell.gripper.close()
    cell.clock.advance(100)
    assert not cell.gripper.is_closed()


# --- cnc ---


def _load_and_start(cell: SimCell) -> None:
    cell.cnc.clamp()
    cell.clock.advance(1)
    cell.cnc.cycle_start()


def test_cnc_normal_cycle(cell: SimCell) -> None:
    assert cell.cnc.door_closed() and not cell.cnc.door_open()
    _load_and_start(cell)
    assert cell.cnc.cycle_running() and not cell.cnc.cycle_done()
    cell.clock.advance(11)
    assert cell.cnc.cycle_done() and not cell.cnc.cycle_running()
    assert not cell.cnc.alarm()
    assert cell.violations == []


def test_cnc_refuses_start_with_door_open(cell: SimCell) -> None:
    cell.cnc.open_door()
    cell.clock.advance(3)
    _load_and_start(cell)
    assert cell.cnc.alarm() and not cell.cnc.cycle_running()


def test_feed_hold_freezes_cycle(cell: SimCell) -> None:
    _load_and_start(cell)
    cell.cnc.feed_hold()
    cell.clock.advance(100)
    assert cell.cnc.feed_hold_active() and not cell.cnc.cycle_done()
    assert not cell.cnc.spindle_cutting()


def test_cnc_faults(cell: SimCell) -> None:
    cell.cnc.inject(CncFault.DOOR_STUCK)
    cell.cnc.open_door()
    cell.clock.advance(100)
    assert not cell.cnc.door_open() and not cell.cnc.door_closed()


def test_clamp_fail_and_cycle_hang(cell: SimCell) -> None:
    cell.cnc.inject(CncFault.CYCLE_HANG)
    _load_and_start(cell)
    cell.clock.advance(1000)
    assert cell.cnc.cycle_running() and not cell.cnc.cycle_done()
    cell.cnc.inject(CncFault.CLAMP_FAIL)
    assert not cell.cnc.clamped()


# --- safety ---


def test_estop_stops_robot_and_machine_in_hardware(cell: SimCell) -> None:
    _load_and_start(cell)
    cell.robot.move_to("above_raw_tray")
    cell.safety.press_estop()
    assert not cell.safety.estop_ok()
    assert cell.robot.status() is RobotStatus.STOPPED
    assert cell.cnc.feed_hold_active()


def test_guard_open_trips(cell: SimCell) -> None:
    cell.safety.open_guard()
    assert not cell.safety.guard_closed()
    assert cell.cnc.feed_hold_active()


def test_safety_interface_is_read_only() -> None:
    public = {n for n in dir(SafetyInputs) if not n.startswith("_")}
    assert public == {"estop_ok", "guard_closed"}


# --- sensors ---


def test_sensors_follow_spindle(cell: SimCell) -> None:
    idle = cell.sensors.read()
    assert idle is not None and idle.spindle_current_a < 3
    _load_and_start(cell)
    cut = cell.sensors.read()
    assert cut is not None and cut.spindle_current_a > CUT_CURRENT_A * 0.8


@pytest.mark.parametrize(
    ("fault", "check"),
    [
        (SensorFault.TOOL_BREAK, lambda a: a < CUT_CURRENT_A * 0.5),
        (SensorFault.JAM, lambda a: a > 25),
    ],
)
def test_sensor_faults(cell: SimCell, fault: SensorFault, check: object) -> None:
    _load_and_start(cell)
    cell.sensors.inject(fault)
    frame = cell.sensors.read()
    assert frame is not None and callable(check) and check(frame.spindle_current_a)


def test_chip_buildup_drifts_up() -> None:
    cell = build_sim_cell(load_cell_config(SIM_CELL), cycle_s=1000.0)
    _load_and_start(cell)
    cell.sensors.inject(SensorFault.CHIP_BUILDUP)
    cell.clock.advance(60)
    frame = cell.sensors.read()
    assert frame is not None and frame.spindle_current_a > CUT_CURRENT_A * 1.4


def test_stale_and_dead_sensors(cell: SimCell) -> None:
    first = cell.sensors.read()
    cell.sensors.inject(SensorFault.STALE)
    cell.clock.advance(5)
    assert cell.sensors.read() == first
    cell.sensors.inject(SensorFault.DEAD)
    assert cell.sensors.read() is None


# --- violation detection (so later tests can trust an empty list) ---


def test_sim_flags_arm_entering_closed_machine(cell: SimCell) -> None:
    cell.robot.move_to("above_fixture")
    assert any("door not open" in v for v in cell.violations)


def test_sim_flags_door_closing_on_arm(cell: SimCell) -> None:
    cell.cnc.open_door()
    cell.clock.advance(3)
    cell.robot.move_to("above_fixture")
    cell.clock.advance(10)
    assert cell.violations == []
    cell.cnc.close_door()
    assert any("door closed on arm" in v for v in cell.violations)


def test_sim_flags_cycle_start_with_arm_inside(cell: SimCell) -> None:
    cell.cnc.open_door()
    cell.clock.advance(3)
    cell.robot.move_to("above_fixture")
    cell.clock.advance(10)
    cell.cnc.cycle_start()
    assert any("arm in machine" in v for v in cell.violations)


def test_gantry_sim_uses_same_interface() -> None:
    from tests.test_config import GANTRY_POSES

    raw = load_cell_config(SIM_CELL).model_dump()
    raw["robot"] = {"kind": "gantry"}
    raw["poses"] = GANTRY_POSES
    cell = build_sim_cell(CellConfig.model_validate(raw))
    robot: Robot = cell.robot
    robot.move_to("tray_slot_3")
    cell.clock.advance(10)
    assert robot.at_pose() == "tray_slot_3"
    robot.move_to("safe_home")
    cell.clock.advance(10)
    assert robot.at_pose() == "safe_home"
