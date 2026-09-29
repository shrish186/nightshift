"""Simulated gripper."""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum

from cell.clock import Clock

# ASSUMPTION: pneumatic parallel gripper, ~0.3 s stroke. Verify on the chosen gripper.
ACTUATE_S = 0.3


class GripperFault(Enum):
    EMPTY_GRIP = "empty_grip"  # closes but misses the part
    DROP = "drop"  # part slips out while held
    STUCK = "stuck"  # jaws never finish moving


class SimGripper:
    def __init__(self, clock: Clock, part_at_tool: Callable[[], bool]) -> None:
        self._clock = clock
        self._part_at_tool = part_at_tool
        self._closed_cmd = False
        self._done_at = 0.0
        self._has_part = False
        self._stuck = False
        self._empty_grip = False
        # Wired by SimCell: told when a held part is released, and when a grip succeeds.
        self.on_release: Callable[[], None] = lambda: None
        self.on_grip: Callable[[], None] = lambda: None

    def _settled(self) -> bool:
        return not self._stuck and self._clock.now() >= self._done_at

    def open(self) -> None:
        if self.has_part():
            self.on_release()
        self._closed_cmd = False
        self._has_part = False
        self._done_at = self._clock.now() + ACTUATE_S

    def close(self) -> None:
        self._closed_cmd = True
        self._has_part = self._part_at_tool() and not self._empty_grip
        self._done_at = self._clock.now() + ACTUATE_S
        if self._has_part:
            self.on_grip()

    def is_open(self) -> bool:
        return not self._closed_cmd and self._settled()

    def is_closed(self) -> bool:
        return self._closed_cmd and self._settled()

    def has_part(self) -> bool:
        return self.is_closed() and self._has_part

    # --- sim only ---
    def inject(self, fault: GripperFault) -> None:
        if fault is GripperFault.EMPTY_GRIP:
            self._empty_grip = True
        elif fault is GripperFault.DROP:
            self._has_part = False
        elif fault is GripperFault.STUCK:
            self._stuck = True
