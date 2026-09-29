"""Simulated watchman sensors, driven by whether the simulated spindle is cutting."""

from __future__ import annotations

import random
from collections.abc import Callable
from enum import Enum

from cell.clock import Clock
from cell.drivers.sensors import SensorFrame

# ASSUMPTIONS: rough signal levels for a mid-size VMC doing steel. M1 data replaces these.
IDLE_CURRENT_A = 1.0
CUT_CURRENT_A = 12.0
CURRENT_NOISE_A = 0.3
IDLE_VIB_G = 0.05
CUT_VIB_G = 0.8
IDLE_AUDIO = 0.1
CUT_AUDIO = 0.6
TOOL_BREAK_FACTOR = 0.25  # cutting current collapses when the tool breaks
JAM_CURRENT_A = 30.0
JAM_VIB_G = 5.0
CHIP_DRIFT_PER_S = 0.01  # fraction of baseline added per second of chip buildup
CHIP_DRIFT_MAX = 2.0


class SensorFault(Enum):
    TOOL_BREAK = "tool_break"
    JAM = "jam"
    CHIP_BUILDUP = "chip_buildup"
    STALE = "stale"  # sensor freezes and keeps returning its last frame
    DEAD = "dead"  # sensor returns nothing


class SimSensors:
    def __init__(self, clock: Clock, cutting: Callable[[], bool], seed: int = 0) -> None:
        self._clock = clock
        self._cutting = cutting
        self._rng = random.Random(seed)
        self._faults: dict[SensorFault, float] = {}  # fault -> time injected
        self._last: SensorFrame | None = None

    def read(self) -> SensorFrame | None:
        if SensorFault.DEAD in self._faults:
            return None
        if SensorFault.STALE in self._faults and self._last is not None:
            return self._last
        self._last = self._sample()
        return self._last

    def _sample(self) -> SensorFrame:
        now = self._clock.now()
        noise = self._rng.gauss(0.0, CURRENT_NOISE_A)
        if not self._cutting():
            return SensorFrame(now, IDLE_CURRENT_A + noise, IDLE_VIB_G, IDLE_AUDIO)
        if SensorFault.JAM in self._faults:
            return SensorFrame(now, JAM_CURRENT_A + noise, JAM_VIB_G, CUT_AUDIO * 2)
        current = CUT_CURRENT_A
        if SensorFault.CHIP_BUILDUP in self._faults:
            elapsed = now - self._faults[SensorFault.CHIP_BUILDUP]
            current *= min(CHIP_DRIFT_MAX, 1.0 + CHIP_DRIFT_PER_S * elapsed)
        if SensorFault.TOOL_BREAK in self._faults:
            current *= TOOL_BREAK_FACTOR
        return SensorFrame(now, current + noise, CUT_VIB_G, CUT_AUDIO)

    # --- sim only ---
    def inject(self, fault: SensorFault) -> None:
        self._faults.setdefault(fault, self._clock.now())
