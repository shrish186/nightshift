"""Simulated CNC machine I/O. Models door, clamp, cycle, feed hold and alarm timing."""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum

from cell.clock import Clock

# ASSUMPTIONS for a typical auto-door VMC with a pneumatic vise. Verify per machine.
DOOR_S = 2.0
CLAMP_S = 0.5
DEFAULT_CYCLE_S = 60.0


class CncFault(Enum):
    DOOR_STUCK = "door_stuck"  # door stops part-way; neither open nor closed sensor
    CLAMP_FAIL = "clamp_fail"  # clamp never confirms
    ALARM = "alarm"  # machine raises an alarm
    CYCLE_HANG = "cycle_hang"  # cycle never reports done


class SimCnc:
    def __init__(self, clock: Clock, cycle_s: float = DEFAULT_CYCLE_S) -> None:
        self._clock = clock
        self._cycle_s = cycle_s
        self._door_target_open = False
        self._door_done_at = 0.0
        self._door_stuck = False
        self._clamp_cmd = False
        self._clamp_done_at = 0.0
        self._clamp_fail = False
        self._in_cycle = False
        self._cycle_end_at = 0.0
        self._cycle_hang = False
        self._done = False
        self._hold = False
        self._alarm = False
        # Wired by SimCell: True if the arm is in the machine envelope.
        self.arm_in_machine: Callable[[], bool] = lambda: False
        self.violations: list[str] = []

    def _update(self) -> None:
        if (
            self._in_cycle
            and not self._hold
            and not self._cycle_hang
            and self._clock.now() >= self._cycle_end_at
        ):
            self._in_cycle = False
            self._done = True

    # --- outputs ---
    def open_door(self) -> None:
        if self._in_cycle:
            self._alarm = True  # real machines refuse; model as alarm
            return
        self._door_target_open = True
        self._door_done_at = self._clock.now() + DOOR_S

    def close_door(self) -> None:
        if self.arm_in_machine():
            self.violations.append(f"t={self._clock.now():.2f} door closed on arm")
        self._door_target_open = False
        self._door_done_at = self._clock.now() + DOOR_S

    def clamp(self) -> None:
        self._clamp_cmd = True
        self._clamp_done_at = self._clock.now() + CLAMP_S

    def unclamp(self) -> None:
        if self._in_cycle:
            self.violations.append(f"t={self._clock.now():.2f} unclamp during cycle")
        self._clamp_cmd = False
        self._clamp_done_at = self._clock.now() + CLAMP_S

    def cycle_start(self) -> None:
        self._update()
        if self.arm_in_machine():
            self.violations.append(f"t={self._clock.now():.2f} cycle start with arm in machine")
        if not (self.door_closed() and self.clamped()) or self._alarm or self._hold:
            self._alarm = True  # real controls refuse to start; model as alarm
            return
        self._done = False
        self._in_cycle = True
        self._cycle_end_at = self._clock.now() + self._cycle_s

    def feed_hold(self) -> None:
        self._hold = True

    # --- inputs ---
    def door_open(self) -> bool:
        return (
            self._door_target_open
            and not self._door_stuck
            and self._clock.now() >= self._door_done_at
        )

    def door_closed(self) -> bool:
        return (
            not self._door_target_open
            and not self._door_stuck
            and self._clock.now() >= self._door_done_at
        )

    def clamped(self) -> bool:
        return self._clamp_cmd and not self._clamp_fail and self._clock.now() >= self._clamp_done_at

    def unclamped(self) -> bool:
        return not self._clamp_cmd and self._clock.now() >= self._clamp_done_at

    def cycle_running(self) -> bool:
        self._update()
        return self._in_cycle

    def cycle_done(self) -> bool:
        self._update()
        return self._done

    def feed_hold_active(self) -> bool:
        return self._hold

    def alarm(self) -> bool:
        return self._alarm

    # --- sim only ---
    def inject(self, fault: CncFault) -> None:
        if fault is CncFault.DOOR_STUCK:
            self._door_stuck = True
        elif fault is CncFault.CLAMP_FAIL:
            self._clamp_fail = True
        elif fault is CncFault.ALARM:
            self._alarm = True
        elif fault is CncFault.CYCLE_HANG:
            self._cycle_hang = True

    def spindle_cutting(self) -> bool:
        return self.cycle_running() and not self._hold

    def operator_clear(self) -> None:
        """A human at the machine clears alarm and feed hold."""
        self._alarm = False
        self._hold = False
