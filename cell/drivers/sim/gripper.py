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
    STUCK = "stuck"  # jaws jam where they are (open, closed or mid-stroke); commands do nothing


class SimGripper:
    def __init__(self, clock: Clock, part_at_tool: Callable[[], bool]) -> None:
        self._clock = clock
        self._part_at_tool = part_at_tool
        self._closed_cmd = False
        self._done_at = 0.0
        self._has_part = False
        self._frozen: str | None = None  # physical jaw position once jammed
        self._empty_grip = False
        # Wired by SimCell: told when a held part is released, and when a grip succeeds.
        self.on_release: Callable[[], None] = lambda: None
        self.on_grip: Callable[[], None] = lambda: None

    def jaws(self) -> str:
        """Physical jaw position: "open", "closed" or "moving"."""
        if self._frozen is not None:
            return self._frozen
        target = "closed" if self._closed_cmd else "open"
        return target if self._clock.now() >= self._done_at else "moving"

    def open(self) -> None:
        if self._frozen is not None:
            return  # jammed: the command has no physical effect
        if self.has_part():
            self.on_release()
        self._closed_cmd = False
        self._has_part = False
        self._done_at = self._clock.now() + ACTUATE_S

    def close(self) -> None:
        if self._frozen is not None:
            return  # jammed: the command has no physical effect
        self._closed_cmd = True
        self._has_part = self._part_at_tool() and not self._empty_grip
        self._done_at = self._clock.now() + ACTUATE_S
        if self._has_part:
            self.on_grip()

    def is_open(self) -> bool:
        return self.jaws() == "open"

    def is_closed(self) -> bool:
        return self.jaws() == "closed"

    def has_part(self) -> bool:
        return self.is_closed() and self._has_part

    # --- sim only ---
    def inject(self, fault: GripperFault) -> None:
        if fault is GripperFault.EMPTY_GRIP:
            self._empty_grip = True
        elif fault is GripperFault.DROP:
            self._has_part = False
        elif fault is GripperFault.STUCK:
            self._frozen = self.jaws()
