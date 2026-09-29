"""Wires the simulated drivers into one physically consistent cell."""

from __future__ import annotations

from dataclasses import dataclass, field

from cell.clock import SimClock
from cell.config import CellConfig
from cell.drivers.sim.cnc import DEFAULT_CYCLE_S, SimCnc
from cell.drivers.sim.gripper import SimGripper
from cell.drivers.sim.robot import SPEED_MM_S, SimRobot
from cell.drivers.sim.safety import SimSafety
from cell.drivers.sim.sensors import SimSensors

# ASSUMPTION: force needed to pull a part out of closed jaws. Far above any sane pull limit.
CLAMP_HOLD_N = 500.0


@dataclass
class SimCell:
    cfg: CellConfig
    clock: SimClock
    robot: SimRobot
    gripper: SimGripper
    cnc: SimCnc
    safety: SimSafety
    sensors: SimSensors
    robot_violations: list[str] = field(default_factory=list)
    interlock_blocks: int = 0  # moves the hardware door-zone interlock refused

    @property
    def violations(self) -> list[str]:
        """Every physically unsafe event the sim observed. Must be empty in every test."""
        return [*self.robot_violations, *self.cnc.violations]


def build_sim_cell(
    cfg: CellConfig,
    seed: int = 0,
    cycle_s: float = DEFAULT_CYCLE_S,
    door_s: float | None = None,
    clamp_s: float | None = None,
    robot_speed_scale: float = 1.0,
    zone_interlock: bool | None = None,
) -> SimCell:
    """Build a sim cell. Door/clamp stroke times default to the config's measured travel."""
    clock = SimClock()
    robot = SimRobot(clock, cfg.poses, speed_mm_s=SPEED_MM_S * robot_speed_scale)
    cnc = SimCnc(
        clock,
        cycle_s=cycle_s,
        clamp_released_sensor=cfg.machine.clamp_released_sensor,
        door_s=cfg.io_plausibility.door_travel_s if door_s is None else door_s,
        clamp_s=cfg.io_plausibility.clamp_travel_s if clamp_s is None else clamp_s,
    )
    # A part can be gripped from the raw tray, or from the fixture if one is there.
    gripper = SimGripper(
        clock,
        part_at_tool=lambda: (
            robot.at_pose() == "pick_raw" or (robot.at_pose() == "load" and cnc.fixture_has_part())
        ),
    )
    safety = SimSafety()
    sensors = SimSensors(clock, cutting=cnc.spindle_cutting, seed=seed)
    cell = SimCell(cfg, clock, robot, gripper, cnc, safety, sensors)

    zone = cfg.machine_zone_poses
    cnc.arm_in_machine = lambda: robot.occupies(zone)
    cnc.arm_holding_at_load = lambda: gripper.has_part() and robot.at_pose() == "load"

    def released() -> None:
        if robot.at_pose() == "load":
            if cnc.jaws() != "closed":
                cell.robot_violations.append(
                    f"t={clock.now():.2f} part released before the clamp closed"
                )
            cnc.place_part()

    def gripped() -> None:
        if robot.at_pose() == "load":
            cnc.take_part()

    gripper.on_release = released
    gripper.on_grip = gripped

    # The jaws grip only near fully closed (see SimCnc.jaw_contact): true at the end of a
    # clamp stroke and the start of an unclamp stroke; not when jammed half-way.
    def holding_clamped_part() -> bool:
        return gripper.has_part() and cnc.jaw_contact()

    def check_move(origin: str, target: str, force_limit_n: float | None) -> None:
        now = f"t={clock.now():.2f}"
        # Physical truth, not sensor readings: a lying sensor must not hide a violation.
        if target in zone and not (cnc.door() == "open" and not cnc.spindle_running()):
            cell.robot_violations.append(
                f"{now} arm moving to {target} with door not open or spindle running"
            )
        # Driving closed, empty jaws onto a part sitting in the fixture crashes into it.
        if (
            target == "load"
            and not gripper.is_open()
            and not gripper.has_part()
            and cnc.fixture_has_part()
        ):
            cell.robot_violations.append(f"{now} gripper closed while entering occupied fixture")
        # An unlimited move away from the fixture while the part is still clamped is a
        # crash. A force-limited move is the approved fallback; its limit catches it.
        if origin == "load" and force_limit_n is None and holding_clamped_part():
            cell.robot_violations.append(f"{now} pulled part from {origin} while still clamped")

    def resistance(origin: str | None) -> float:
        return CLAMP_HOLD_N if origin == "load" and holding_clamped_part() else 0.0

    # Hardware door-zone interlock (safety relay): refuses any move into the machine zone
    # unless the door is physically fully open and the interlock channel is healthy.
    # zone_interlock=False models the software alone, for tests of the software layer.
    fitted = cfg.safety.door_zone_interlock if zone_interlock is None else zone_interlock

    def permit(origin: str, target: str) -> bool:
        if not fitted or target not in zone:
            return True
        if cnc.door() == "open" and safety.zone_interlock_ok():
            return True
        cell.interlock_blocks += 1
        return False

    robot.hardware_permit = permit
    robot.on_move_start = check_move
    robot.resistance_n = resistance

    # Hardware e-stop/guard chain stops robot and machine independently of software.
    safety.on_trip.append(robot.stop)
    safety.on_trip.append(cnc.feed_hold)
    return cell
