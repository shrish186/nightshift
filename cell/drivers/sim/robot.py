"""Simulated robot arm. Motion time is computed from pose distance on the sim clock."""

from __future__ import annotations

import math
from collections.abc import Callable
from enum import Enum

from cell.clock import Clock
from cell.config import Pose
from cell.drivers.robot import RobotStatus

# ASSUMPTION: ~250 mm/s effective cartesian speed for a small cobot doing tending moves
# with conservative settings. Verify on the chosen arm.
SPEED_MM_S = 250.0
MIN_MOVE_S = 0.5


class RobotFault(Enum):
    FAULT = "fault"  # robot controller faults (e.g. joint limit, comms loss)
    STALL = "stall"  # motion never completes (e.g. collision detection holding it)


class SimRobot:
    def __init__(self, clock: Clock, poses: dict[str, Pose], start_pose: str = "home") -> None:
        self._clock = clock
        self._poses = poses
        self._pose: str | None = start_pose
        self._origin: str | None = None
        self._target: str | None = None
        self._arrive_at = 0.0
        self._status = RobotStatus.IDLE
        self._stall = False
        self._force_limit_n: float | None = None
        # Wired by SimCell: called as (origin, target, force_limit_n) when a move starts,
        # to record unsafe moves.
        self.on_move_start: Callable[[str, str, float | None], None] | None = None
        # Wired by SimCell: resistance in newtons felt on a move leaving `origin`.
        self.resistance_n: Callable[[str | None], float] = lambda origin: 0.0

    def _update(self) -> None:
        if (
            self._status is RobotStatus.MOVING
            and self._force_limit_n is not None
            and self.resistance_n(self._origin) > self._force_limit_n
        ):
            self._status = RobotStatus.FORCE_LIMIT  # stopped mid-move; origin/target kept
            return
        if (
            self._status is RobotStatus.MOVING
            and not self._stall
            and self._clock.now() >= self._arrive_at
        ):
            self._pose, self._origin, self._target = self._target, None, None
            self._status = RobotStatus.IDLE
            self._force_limit_n = None

    def move_to(self, pose: str) -> None:
        self._start_move(pose, None)

    def move_to_limited(self, pose: str, force_limit_n: float) -> None:
        self._start_move(pose, force_limit_n)

    def _start_move(self, pose: str, force_limit_n: float | None) -> None:
        if pose not in self._poses:
            raise ValueError(f"unknown pose {pose!r}")
        self._update()
        if self._status is not RobotStatus.IDLE or self._pose is None:
            return
        if self.on_move_start is not None:
            self.on_move_start(self._pose, pose, force_limit_n)
        self._force_limit_n = force_limit_n
        dist = math.dist(self._poses[self._pose][:3], self._poses[pose][:3])
        self._origin, self._target = self._pose, pose
        self._pose = None
        self._arrive_at = self._clock.now() + max(MIN_MOVE_S, dist / SPEED_MM_S)
        self._status = RobotStatus.MOVING
        self._update()  # a limited move into immediate resistance trips at once

    def stop(self) -> None:
        self._update()
        if self._status not in (RobotStatus.FAULT, RobotStatus.FORCE_LIMIT):
            self._status = RobotStatus.STOPPED

    def status(self) -> RobotStatus:
        self._update()
        return self._status

    def at_pose(self) -> str | None:
        self._update()
        return self._pose if self._status is not RobotStatus.MOVING else None

    def reset(self) -> None:
        self._update()
        if self._status in (RobotStatus.STOPPED, RobotStatus.FAULT, RobotStatus.FORCE_LIMIT):
            self._status = RobotStatus.IDLE
            self._stall = False
            self._force_limit_n = None

    # --- sim only ---
    def inject(self, fault: RobotFault) -> None:
        self._update()
        if fault is RobotFault.FAULT:
            self._status = RobotStatus.FAULT
        elif fault is RobotFault.STALL:
            self._stall = True

    def occupies(self, zone: frozenset[str]) -> bool:
        """True if the arm is at, leaving, or heading to a pose in the zone.

        A move stopped part-way keeps its origin/target, so a stop inside the machine
        still counts as occupying it.
        """
        self._update()
        return any(p in zone for p in (self._pose, self._origin, self._target) if p)
