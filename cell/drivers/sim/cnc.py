"""Simulated CNC machine I/O. Models door, clamp, cycle, feed hold and alarm timing.

The sim keeps *physical truth* (where the door and jaws really are, whether the
spindle is really running) separate from *sensor readings* (what the CncIo inputs
report). Sensor faults change only the readings. The unsafe-event log always checks
physical truth, so a lying sensor cannot hide an unsafe event from the tests.
"""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum

from cell.clock import Clock

# ASSUMPTIONS for a typical auto-door VMC with a pneumatic vise. Verify per machine.
DOOR_S = 2.0
CLAMP_S = 0.5
DEFAULT_CYCLE_S = 60.0


class CncFault(Enum):
    DOOR_STUCK = "door_stuck"  # door stops part-way on its current/next move
    CLAMP_FAIL = "clamp_fail"  # jaws never close
    ALARM = "alarm"  # machine raises an alarm
    CYCLE_HANG = "cycle_hang"  # cycle never reports done
    CLAMP_STUCK_ON = "clamp_stuck_on"  # jaws stay closed even when unclamp is commanded
    CLAMP_JAM = "clamp_jam"  # jaws jam half-way on their current/next stroke
    DOOR_SENSOR_SHORT = "door_sensor_short"  # both door sensors read true
    CLAMP_SENSOR_SHORT = "clamp_sensor_short"  # both clamp sensors read true
    IO_POWER_LOSS = "io_power_loss"  # I/O loses power: every wire reads de-energised
    # Door starts closing on its own: operator hits the door button, or air pressure drops.
    DOOR_CLOSES_UNCOMMANDED = "door_closes_uncommanded"


class SimCnc:
    def __init__(
        self,
        clock: Clock,
        cycle_s: float = DEFAULT_CYCLE_S,
        clamp_released_sensor: bool = True,
        door_s: float = DOOR_S,
        clamp_s: float = CLAMP_S,
    ) -> None:
        self._clock = clock
        self._door_s = door_s
        self._clamp_s = clamp_s
        self._cycle_s = cycle_s
        self._has_released_sensor = clamp_released_sensor
        self._door_target_open = False
        self._door_done_at = 0.0
        self._door_frozen: str | None = None
        self._door_stick_next = False
        self._clamp_cmd = False
        self._clamp_done_at = 0.0
        self._clamp_fail = False
        self._jaws_frozen: str | None = None
        self._in_cycle = False
        self._cycle_end_at = 0.0
        self._cycle_hang = False
        self._done = False
        self._hold = False
        self._alarm = False
        self._sensor_faults: set[CncFault] = set()
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

    def _violation(self, what: str) -> None:
        self.violations.append(f"t={self._clock.now():.2f} {what}")

    # --- physical truth (sim only) ---
    def door(self) -> str:
        """Physical door position: "open", "closed" or "moving" (includes stuck half-way)."""
        if self._door_frozen is not None:
            return self._door_frozen
        target = "open" if self._door_target_open else "closed"
        return target if self._clock.now() >= self._door_done_at else "moving"

    def jaws(self) -> str:
        """Physical jaw position: "open", "closed" or "moving"."""
        if self._jaws_frozen is not None:
            return self._jaws_frozen
        target = "closed" if self._clamp_cmd and not self._clamp_fail else "open"
        return target if self._clock.now() >= self._clamp_done_at else "moving"

    def spindle_running(self) -> bool:
        self._update()
        return self._in_cycle

    def spindle_cutting(self) -> bool:
        return self.spindle_running() and not self._hold

    # --- outputs ---
    def _move_door(self, to_open: bool) -> None:
        if self._door_frozen is not None:
            return
        self._door_target_open = to_open
        self._door_done_at = self._clock.now() + self._door_s
        if self._door_stick_next:
            self._door_frozen = "moving"

    def open_door(self) -> None:
        if self.spindle_running():
            self._alarm = True  # real machines refuse; model as alarm
            return
        self._move_door(True)

    def close_door(self) -> None:
        if self.arm_in_machine():
            self._violation("door closed on arm")
        self._move_door(False)

    def clamp(self) -> None:
        self._clamp_cmd = True
        self._clamp_done_at = self._clock.now() + self._clamp_s

    def unclamp(self) -> None:
        if self.spindle_running():
            self._violation("unclamp during cycle")
        self._clamp_cmd = False
        self._clamp_done_at = self._clock.now() + self._clamp_s

    def cycle_start(self) -> None:
        self._update()
        if self.arm_in_machine():
            self._violation("cycle start with arm in machine")
        # The machine's own interlocks use its own (physical) door and clamp state.
        ready = self.door() == "closed" and self.jaws() == "closed"
        if not ready or self._alarm or self._hold:
            self._alarm = True  # real controls refuse to start; model as alarm
            return
        self._done = False
        self._in_cycle = True
        self._cycle_end_at = self._clock.now() + self._cycle_s

    def feed_hold(self) -> None:
        self._hold = True

    # --- inputs (sensor readings) ---
    @property
    def _powered(self) -> bool:
        return CncFault.IO_POWER_LOSS not in self._sensor_faults

    def door_open(self) -> bool:
        if not self._powered:
            return False
        return CncFault.DOOR_SENSOR_SHORT in self._sensor_faults or self.door() == "open"

    def door_closed(self) -> bool:
        if not self._powered:
            return False
        return CncFault.DOOR_SENSOR_SHORT in self._sensor_faults or self.door() == "closed"

    def clamped(self) -> bool:
        if not self._powered:
            return False
        return CncFault.CLAMP_SENSOR_SHORT in self._sensor_faults or self.jaws() == "closed"

    def unclamped(self) -> bool:
        # No sensor fitted: never reads as released (fail-safe; see CncIo.unclamped).
        if not self._has_released_sensor or not self._powered:
            return False
        return CncFault.CLAMP_SENSOR_SHORT in self._sensor_faults or self.jaws() == "open"

    # "Bad when true" inputs are wired inverted (see cnc_io.py), so a dead wire reads True.
    def cycle_running(self) -> bool:
        return not self._powered or self.spindle_running()

    def cycle_done(self) -> bool:
        self._update()
        return self._powered and self._done

    def feed_hold_active(self) -> bool:
        return not self._powered or self._hold

    def alarm(self) -> bool:
        return not self._powered or self._alarm

    # --- sim only ---
    def inject(self, fault: CncFault) -> None:
        if fault is CncFault.DOOR_STUCK:
            if self.door() == "moving":
                self._door_frozen = "moving"
            else:
                self._door_stick_next = True
        elif fault is CncFault.CLAMP_FAIL:
            self._clamp_fail = True
        elif fault is CncFault.ALARM:
            self._alarm = True
        elif fault is CncFault.CYCLE_HANG:
            self._cycle_hang = True
        elif fault is CncFault.CLAMP_STUCK_ON:
            self._jaws_frozen = "closed"
        elif fault is CncFault.CLAMP_JAM:
            self._jaws_frozen = "moving"
        elif fault is CncFault.DOOR_CLOSES_UNCOMMANDED:
            # Physical motion only. Not recorded as a controller violation: stopping a
            # closing door on an arm is the door's own hardware protection. What the
            # controller must do is stop sending the arm deeper in.
            self._move_door(False)
        else:
            self._sensor_faults.add(fault)

    def operator_clear(self) -> None:
        """A human at the machine clears alarm and feed hold."""
        self._alarm = False
        self._hold = False
