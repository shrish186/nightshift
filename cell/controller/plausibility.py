"""Sensor-pair plausibility checks. SAFETY-RELEVANT: changes here need human review.

A door or clamp with two sensors (open + closed, clamped + released) can never
legitimately read both True, and can only read both False while it is travelling
(up to the configured io_plausibility window, which is measured travel + margin).
Anything else means a shorted or broken sensor, a stuck mechanism or lost I/O power.
The controller calls check() every step and goes to SAFE on any fault.

No knowledge of what was commanded is needed: the "both off" timer starts the first
time both sensors are seen False and resets as soon as either reads True.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from cell.clock import Clock
from cell.config import CellConfig
from cell.drivers.cnc_io import CncIo


class PlausibilityFault(Enum):
    DOOR_BOTH_ON = "door_both_on"
    DOOR_BOTH_OFF_TOO_LONG = "door_both_off_too_long"
    CLAMP_BOTH_ON = "clamp_both_on"
    CLAMP_BOTH_OFF_TOO_LONG = "clamp_both_off_too_long"


@dataclass
class _Pair:
    both_on: PlausibilityFault
    both_off: PlausibilityFault
    travel_s: float
    off_since: float | None = None

    def check(self, a: bool, b: bool, now: float) -> PlausibilityFault | None:
        if a and b:
            self.off_since = None
            return self.both_on
        if a or b:
            self.off_since = None
            return None
        if self.off_since is None:
            self.off_since = now
        return self.both_off if now - self.off_since > self.travel_s else None


class IoPlausibilityMonitor:
    def __init__(self, cnc: CncIo, clock: Clock, cfg: CellConfig) -> None:
        self._cnc = cnc
        self._clock = clock
        p = cfg.io_plausibility
        self._door = _Pair(
            PlausibilityFault.DOOR_BOTH_ON,
            PlausibilityFault.DOOR_BOTH_OFF_TOO_LONG,
            p.door_plausibility_window_s,
        )
        self._clamp: _Pair | None = None
        if cfg.machine.clamp_released_sensor:
            self._clamp = _Pair(
                PlausibilityFault.CLAMP_BOTH_ON,
                PlausibilityFault.CLAMP_BOTH_OFF_TOO_LONG,
                p.clamp_plausibility_window_s,
            )

    def check(self) -> list[PlausibilityFault]:
        now = self._clock.now()
        faults = [self._door.check(self._cnc.door_open(), self._cnc.door_closed(), now)]
        if self._clamp is not None:
            faults.append(self._clamp.check(self._cnc.clamped(), self._cnc.unclamped(), now))
        return [f for f in faults if f is not None]
