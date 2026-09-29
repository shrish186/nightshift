"""Robot arm interface. Commands are non-blocking; the controller polls status()."""

from __future__ import annotations

from enum import Enum
from typing import Protocol


class RobotStatus(Enum):
    IDLE = "idle"  # not moving, ready for a command
    MOVING = "moving"
    STOPPED = "stopped"  # protective stop; needs reset() before it will move again
    FAULT = "fault"  # robot-side fault; needs reset()


class Robot(Protocol):
    def move_to(self, pose: str) -> None:
        """Start moving to a named pose from the cell config. Ignored unless IDLE."""

    def stop(self) -> None:
        """Protective stop. Always accepted, from any status."""

    def status(self) -> RobotStatus: ...

    def at_pose(self) -> str | None:
        """Name of the pose the robot is resting at, or None while moving/unknown."""

    def reset(self) -> None:
        """Clear STOPPED/FAULT. Only called from a human-initiated controller reset."""
