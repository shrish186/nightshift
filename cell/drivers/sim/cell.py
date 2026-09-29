"""Wires the simulated drivers into one physically consistent cell."""

from __future__ import annotations

from dataclasses import dataclass, field

from cell.clock import SimClock
from cell.config import CellConfig
from cell.drivers.sim.cnc import DEFAULT_CYCLE_S, SimCnc
from cell.drivers.sim.gripper import SimGripper
from cell.drivers.sim.robot import SimRobot
from cell.drivers.sim.safety import SimSafety
from cell.drivers.sim.sensors import SimSensors

# Poses where the gripper can actually grab a part (raw tray pick, finished part in fixture).
GRIP_POSES = frozenset({"pick_raw", "load"})


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

    @property
    def violations(self) -> list[str]:
        """Every physically unsafe event the sim observed. Must be empty in every test."""
        return [*self.robot_violations, *self.cnc.violations]


def build_sim_cell(cfg: CellConfig, seed: int = 0, cycle_s: float = DEFAULT_CYCLE_S) -> SimCell:
    clock = SimClock()
    robot = SimRobot(clock, cfg.poses)
    gripper = SimGripper(clock, part_at_tool=lambda: robot.at_pose() in GRIP_POSES)
    cnc = SimCnc(clock, cycle_s=cycle_s)
    safety = SimSafety()
    sensors = SimSensors(clock, cutting=cnc.spindle_cutting, seed=seed)
    cell = SimCell(cfg, clock, robot, gripper, cnc, safety, sensors)

    zone = cfg.machine_zone_poses
    cnc.arm_in_machine = lambda: robot.occupies(zone)

    def check_entry(target: str) -> None:
        if target in zone and not (cnc.door_open() and not cnc.cycle_running()):
            cell.robot_violations.append(
                f"t={clock.now():.2f} arm moving to {target} with door not open or spindle running"
            )

    robot.on_move_start = check_entry

    # Hardware e-stop/guard chain stops robot and machine independently of software.
    safety.on_trip.append(robot.stop)
    safety.on_trip.append(cnc.feed_hold)
    return cell
