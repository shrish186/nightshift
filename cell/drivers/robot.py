"""Robot interface. Commands are non-blocking; the controller polls status().

The interface speaks only in *named poses* ("load", "tray_slot_3", "safe_home").
Coordinates, joint angles, axis counts and kinematics live in the cell config and
the driver, never in business logic. That way a cheap 2-3 axis gantry loader and a
6-axis cobot both implement this same interface.
"""

from __future__ import annotations

from enum import Enum
from typing import Protocol


class RobotStatus(Enum):
    IDLE = "idle"  # not moving, ready for a command
    MOVING = "moving"
    STOPPED = "stopped"  # protective stop; needs reset() before it will move again
    FAULT = "fault"  # robot-side fault; needs reset()
    FORCE_LIMIT = "force_limit"  # a force-limited move met too much resistance; needs reset()


class Robot(Protocol):
    def move_to(self, pose: str) -> None:
        """Start moving to a pose, by name, from the cell config. Ignored unless IDLE."""

    def move_to_limited(self, pose: str, force_limit_n: float) -> None:
        """Like move_to, but stop and latch FORCE_LIMIT if resistance exceeds the limit.

        Used for the gentle pull on machines without a clamp-released sensor.
        ASSUMPTION: a cobot does this with its built-in force/torque sensing; a gantry
        with a motor-current limit calibrated to newtons. Verify on hardware.
        """

    def stop(self) -> None:
        """Protective stop. Always accepted, from any status."""

    def status(self) -> RobotStatus: ...

    def at_pose(self) -> str | None:
        """Name of the pose the robot is resting at, or None while moving/unknown."""

    def reset(self) -> None:
        """Clear STOPPED/FAULT/FORCE_LIMIT. Only called from a human-initiated controller reset."""
