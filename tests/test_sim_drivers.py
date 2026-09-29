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
    cell.sensors.read()  # first sample of the cut starts the spindle ramp
    cell.clock.advance(1)
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
    cell.sensors.read()
    cell.clock.advance(1)
    cell.sensors.inject(fault)
    frame = cell.sensors.read()
    assert frame is not None and callable(check) and check(frame.spindle_current_a)


def test_chip_buildup_drifts_up() -> None:
    cell = build_sim_cell(load_cell_config(SIM_CELL), cycle_s=1000.0)
    _load_and_start(cell)
    cell.sensors.read()
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


# --- clamp release: sensor vs. force-limited fallback ---


def _grip_finished_part(cell: SimCell) -> None:
    """Door open, part clamped in the fixture, arm at load with the gripper closed on it."""
    cell.cnc.clamp()
    cell.cnc.open_door()
    cell.clock.advance(3)
    cell.robot.move_to("above_fixture")
    cell.clock.advance(10)
    cell.robot.move_to("load")
    cell.clock.advance(10)
    cell.cnc.place_part()  # a finished part is sitting in the fixture
    cell.gripper.close()
    cell.clock.advance(1)
    assert cell.gripper.has_part() and cell.cnc.clamped()


def _no_sensor_cell() -> SimCell:
    raw = load_cell_config(SIM_CELL).model_dump()
    raw["machine"]["clamp_released_sensor"] = False
    raw["pins"]["unclamped_in"] = None
    raw["unclamp_fallback"] = {
        "release_wait_s": 2.0,
        "pull_pose": "above_fixture",
        "pull_force_limit_n": 40.0,
    }
    return build_sim_cell(CellConfig.model_validate(raw))


def test_release_sensor_confirms_unclamp(cell: SimCell) -> None:
    _grip_finished_part(cell)
    cell.cnc.unclamp()
    assert not cell.cnc.unclamped()
    cell.clock.advance(1)
    assert cell.cnc.unclamped() and not cell.cnc.clamped()


def test_no_sensor_machine_never_reads_released() -> None:
    cell = _no_sensor_cell()
    _grip_finished_part(cell)
    cell.cnc.unclamp()
    cell.clock.advance(100)
    assert cell.cnc.jaws() == "open"  # physically released...
    assert not cell.cnc.unclamped()  # ...but with no sensor it never reads as released


def test_limited_pull_succeeds_once_released() -> None:
    cell = _no_sensor_cell()
    _grip_finished_part(cell)
    cell.cnc.unclamp()
    cell.clock.advance(2)
    cell.robot.move_to_limited("above_fixture", 40.0)
    cell.clock.advance(10)
    assert cell.robot.at_pose() == "above_fixture" and cell.gripper.has_part()
    assert cell.violations == []


def test_limited_pull_trips_when_jaws_stuck_on() -> None:
    cell = _no_sensor_cell()
    _grip_finished_part(cell)
    cell.cnc.inject(CncFault.CLAMP_STUCK_ON)
    cell.cnc.unclamp()
    cell.clock.advance(2)
    cell.robot.move_to_limited("above_fixture", 40.0)
    assert cell.robot.status() is RobotStatus.FORCE_LIMIT
    cell.clock.advance(10)
    assert cell.robot.status() is RobotStatus.FORCE_LIMIT  # latched, not creeping on
    assert cell.violations == []  # the limit did its job
    cell.robot.reset()
    assert cell.robot.status() is RobotStatus.IDLE


def test_stuck_on_clamp_still_reads_clamped_with_sensor(cell: SimCell) -> None:
    _grip_finished_part(cell)
    cell.cnc.inject(CncFault.CLAMP_STUCK_ON)
    cell.cnc.unclamp()
    cell.clock.advance(10)
    assert cell.cnc.clamped() and not cell.cnc.unclamped()


def test_door_closes_uncommanded(cell: SimCell) -> None:
    cell.cnc.open_door()
    cell.clock.advance(3)
    assert cell.cnc.door_open()
    cell.cnc.inject(CncFault.DOOR_CLOSES_UNCOMMANDED)
    assert not cell.cnc.door_open() and cell.cnc.door() == "moving"
    cell.clock.advance(3)
    assert cell.cnc.door_closed()


# --- part in fixture ---


def _arm_at_load_holding_raw(cell: SimCell) -> None:
    cell.robot.move_to("pick_raw")
    cell.clock.advance(10)
    cell.gripper.close()
    cell.clock.advance(1)
    cell.cnc.open_door()
    cell.clock.advance(3)
    cell.robot.move_to("above_fixture")
    cell.clock.advance(10)
    cell.robot.move_to("load")
    cell.clock.advance(10)


def test_part_present_follows_the_part(cell: SimCell) -> None:
    assert not cell.cnc.part_present()
    _arm_at_load_holding_raw(cell)
    assert cell.cnc.part_present()  # held part sitting in the fixture
    cell.gripper.open()
    cell.clock.advance(1)
    assert cell.cnc.part_present() and cell.cnc.seat() == "seated"
    cell.robot.move_to("above_fixture")
    cell.clock.advance(10)
    assert cell.cnc.part_present()  # arm gone, part stays
    cell.robot.move_to("load")
    cell.clock.advance(10)
    cell.gripper.close()
    cell.clock.advance(1)
    cell.robot.move_to("above_fixture")
    cell.clock.advance(10)
    assert cell.gripper.has_part() and not cell.cnc.part_present()


def test_gripper_finds_nothing_in_empty_fixture(cell: SimCell) -> None:
    cell.cnc.open_door()
    cell.clock.advance(3)
    cell.robot.move_to("above_fixture")
    cell.clock.advance(10)
    cell.robot.move_to("load")
    cell.clock.advance(10)
    cell.gripper.close()
    cell.clock.advance(1)
    assert not cell.gripper.has_part()


def test_misseated_part_reads_not_present(cell: SimCell) -> None:
    cell.cnc.inject(CncFault.PART_MISSEATED)
    _arm_at_load_holding_raw(cell)
    assert cell.cnc.seat() == "crooked" and not cell.cnc.part_present()


def test_sim_flags_cycle_start_on_crooked_part(cell: SimCell) -> None:
    _arm_at_load_holding_raw(cell)
    cell.cnc.clamp()
    cell.gripper.open()
    cell.clock.advance(1)
    cell.robot.move_to("clear_of_machine")
    cell.clock.advance(10)
    cell.cnc.close_door()
    cell.clock.advance(3)
    cell.cnc.inject(CncFault.PART_MISSEATED)
    cell.cnc.cycle_start()
    assert any("part not seated" in v for v in cell.violations)


def test_part_present_reads_false_on_power_loss(cell: SimCell) -> None:
    _arm_at_load_holding_raw(cell)
    cell.cnc.inject(CncFault.IO_POWER_LOSS)
    assert not cell.cnc.part_present()


def test_tool_wear_grows_cycle_over_cycle() -> None:
    from cell.drivers.sim.sensors import SimSensors

    clock_cell = build_sim_cell(load_cell_config(SIM_CELL))
    cutting = [False]
    sensors = SimSensors(clock_cell.clock, cutting=lambda: cutting[0], seed=1, noise=False)
    sensors.inject(SensorFault.TOOL_WEAR)
    means = []
    for _ in range(3):
        cutting[0] = True
        sensors.read()
        clock_cell.clock.advance(2)
        frame = sensors.read()
        assert frame is not None
        means.append(frame.spindle_current_a)
        cutting[0] = False
        sensors.read()
    assert means[0] < means[1] < means[2]
    assert means[2] / means[0] == pytest.approx(1.2)


def test_gripper_jammed_open_stays_open(cell: SimCell) -> None:
    cell.clock.advance(1)
    assert cell.gripper.is_open()
    cell.gripper.inject(GripperFault.STUCK)
    assert cell.gripper.is_open()  # still physically open
    cell.gripper.close()
    cell.clock.advance(5)
    assert cell.gripper.is_open() and not cell.gripper.is_closed()
