"""Simulated hardware safety inputs.

The press_/release_/open_/close_ methods stand for a *person physically* hitting the
e-stop or opening a guard. They exist only in the sim; production code gets a
SafetyInputs, which is read-only. The hardware effect of a trip (robot and machine
stopping regardless of software) is modelled by SimCell, not here.
"""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum


class SafetyFault(Enum):
    ZONE_INTERLOCK_FAULT = "zone_interlock_fault"  # relay detects a switch/channel fault


class SimSafety:
    def __init__(self) -> None:
        self._estop_ok = True
        self._guard_closed = True
        self._zone_ok = True
        self.on_trip: list[Callable[[], None]] = []

    def estop_ok(self) -> bool:
        return self._estop_ok

    def guard_closed(self) -> bool:
        return self._guard_closed

    def zone_interlock_ok(self) -> bool:
        return self._zone_ok

    # --- sim only ---
    def inject(self, fault: SafetyFault) -> None:
        if fault is SafetyFault.ZONE_INTERLOCK_FAULT:
            self._zone_ok = False  # a faulted safety channel fails safe: blocks entry

    # --- sim only: physical human actions ---
    def press_estop(self) -> None:
        self._estop_ok = False
        self._trip()

    def release_estop(self) -> None:
        self._estop_ok = True

    def open_guard(self) -> None:
        self._guard_closed = False
        self._trip()

    def close_guard(self) -> None:
        self._guard_closed = True

    def _trip(self) -> None:
        for cb in self.on_trip:
            cb()
