"""Simulated watchman sensors, driven by whether the simulated spindle is cutting.

Realistic noise is on by default so the watchman is tested against it: per-cycle load
variation, gaussian noise on every channel, rare single-sample spikes and a spindle
ramp-up at the start of each cut.
"""

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
VIB_NOISE_G = 0.05
IDLE_AUDIO = 0.1
CUT_AUDIO = 0.6
AUDIO_NOISE = 0.03
CYCLE_LOAD_SD = 0.03  # cycle-to-cycle load variation (stock size, hardness)
CYCLE_LOAD_CLIP = 0.08
SPIKE_PROB = 1 / 500  # e.g. a chip hitting the tool: one sample, not sustained
SPIKE_CURRENT = 1.5
SPIKE_VIB = 2.0
RAMP_S = 0.5  # spindle/feed ramp-up at the start of each cut

TOOL_BREAK_FACTOR = 0.25  # cutting current collapses when the tool breaks
JAM_CURRENT_A = 30.0
JAM_VIB_G = 5.0
CHIP_DRIFT_PER_S = 0.05  # fraction of load added per second of cutting within a cycle
CHIP_DRIFT_MAX = 2.0
WEAR_PER_CYCLE = 0.10  # fraction of load added per completed cycle once wear starts


class SensorFault(Enum):
    TOOL_BREAK = "tool_break"
    JAM = "jam"
    CHIP_BUILDUP = "chip_buildup"  # load climbs during each cut; resets next cycle
    TOOL_WEAR = "tool_wear"  # load climbs cycle over cycle
    STALE = "stale"  # sensor freezes and keeps returning its last frame
    DEAD = "dead"  # sensor returns nothing


class SimSensors:
    def __init__(
        self, clock: Clock, cutting: Callable[[], bool], seed: int = 0, noise: bool = True
    ) -> None:
        self._clock = clock
        self._cutting = cutting
        self._rng = random.Random(seed)
        self._noise = noise
        self._faults: dict[SensorFault, float] = {}  # fault -> time injected
        self._last: SensorFrame | None = None
        self._was_cutting = False
        self._cut_start = 0.0
        self._cycle_load = 1.0
        self._cycles_since_wear = 0

    def read(self) -> SensorFrame | None:
        if SensorFault.DEAD in self._faults:
            return None
        if SensorFault.STALE in self._faults and self._last is not None:
            return self._last
        self._last = self._sample()
        return self._last

    def _gauss(self, sd: float) -> float:
        return self._rng.gauss(0.0, sd) if self._noise else 0.0

    def _track_cycles(self, cutting: bool, now: float) -> None:
        if cutting and not self._was_cutting:
            self._cut_start = now
            load = 1.0 + self._gauss(CYCLE_LOAD_SD)
            self._cycle_load = min(1 + CYCLE_LOAD_CLIP, max(1 - CYCLE_LOAD_CLIP, load))
        if not cutting and self._was_cutting and SensorFault.TOOL_WEAR in self._faults:
            self._cycles_since_wear += 1
        self._was_cutting = cutting

    def _sample(self) -> SensorFrame:
        now = self._clock.now()
        cutting = self._cutting()
        self._track_cycles(cutting, now)
        if not cutting:
            return SensorFrame(
                now,
                IDLE_CURRENT_A + self._gauss(CURRENT_NOISE_A),
                IDLE_VIB_G + abs(self._gauss(VIB_NOISE_G)),
                IDLE_AUDIO + abs(self._gauss(AUDIO_NOISE)),
            )
        if SensorFault.JAM in self._faults:
            return SensorFrame(now, JAM_CURRENT_A + self._gauss(CURRENT_NOISE_A), JAM_VIB_G, 1.2)

        in_cut = now - self._cut_start
        load = self._cycle_load * min(1.0, in_cut / RAMP_S) if RAMP_S > 0 else self._cycle_load
        if SensorFault.CHIP_BUILDUP in self._faults:
            since = now - max(self._cut_start, self._faults[SensorFault.CHIP_BUILDUP])
            load *= min(CHIP_DRIFT_MAX, 1.0 + CHIP_DRIFT_PER_S * max(0.0, since))
        if SensorFault.TOOL_WEAR in self._faults:
            load *= 1.0 + WEAR_PER_CYCLE * self._cycles_since_wear
        if SensorFault.TOOL_BREAK in self._faults:
            load *= TOOL_BREAK_FACTOR

        current = CUT_CURRENT_A * load
        vib = CUT_VIB_G * min(load, 1.0)
        if self._noise and self._rng.random() < SPIKE_PROB:
            current *= SPIKE_CURRENT
            vib *= SPIKE_VIB
        return SensorFrame(
            now,
            max(0.0, current + self._gauss(CURRENT_NOISE_A)),
            max(0.0, vib + self._gauss(VIB_NOISE_G)),
            CUT_AUDIO + abs(self._gauss(AUDIO_NOISE)),
        )

    # --- sim only ---
    def inject(self, fault: SensorFault) -> None:
        self._faults.setdefault(fault, self._clock.now())

    def clear(self, fault: SensorFault) -> None:
        """The fault goes away (e.g. the node comes back online)."""
        self._faults.pop(fault, None)
        if fault is SensorFault.STALE:
            self._last = None
