"""Simulated STANDALONE machine (no robot, no CNC I/O): the December pilot setup.

Produces the node's feature stream (spindle current RMS, vibration RMS) for realistic
cycles, so standalone-mode watchman logic is tested the way the pilots will run it:

  spindle start (ramp) -> approach (spindle idle, no load) -> cut (load) ->
  retract (spindle idle) -> spindle idle -> spindle stop

with per-cycle timing jitter, load variation, noise and rare spikes. Faults: a tool
break at a chosen fraction of the cut (the program keeps running: the broken tool
air-cuts), a tool broken from the start (air cut), and a jam.

ASSUMPTIONS (replace with makerspace data, W3): current levels, ramp times, jitter.
"""

from __future__ import annotations

import random
from collections.abc import Callable
from dataclasses import dataclass

from cell.clock import SimClock
from cell.drivers.sensors import SensorFrame

DT = 0.1
SPINDLE_OFF_A = 0.3
SPINDLE_IDLE_A = 2.5  # spindle turning, no load
AIR_CUT_A = 3.0  # spindle + feed, broken tool not touching the part
CUT_A = 12.0
JAM_A = 30.0
OFF_VIB, IDLE_VIB, CUT_VIB, JAM_VIB = 0.02, 0.15, 0.8, 5.0
LOW_NOISE_A, CUT_NOISE_A, VIB_NOISE = 0.08, 0.3, 0.04
RAMP_S = 0.5
SPIKE_PROB = 1 / 500


@dataclass(frozen=True)
class CyclePlan:
    approach_s: float
    cut_s: float
    retract_s: float
    idle_s: float
    stop_s: float
    load: float  # per-cycle load factor
    break_at: float | None = None  # fraction of the cut where the tool breaks
    air_cut: bool = False  # tool already broken/missing: the whole cut is an air cut
    jam_at: float | None = None  # fraction of the cut where the tool jams

    @property
    def load_start_s(self) -> float:
        return RAMP_S + self.approach_s


class StandaloneMachine:
    def __init__(self, seed: int = 0, nominal_cut_s: float = 12.0, noise: bool = True) -> None:
        self.clock = SimClock()
        self._rng = random.Random(seed)
        self.nominal_cut_s = nominal_cut_s
        self._noise = noise

    def plan(
        self,
        break_at: float | None = None,
        air_cut: bool = False,
        jam_at: float | None = None,
    ) -> CyclePlan:
        r = self._rng
        return CyclePlan(
            approach_s=r.uniform(1.0, 3.0),
            # A CNC program's cut time is fixed by its feeds: it repeats within ~+/-2%
            # (ASSUMPTION; an operator changing feed override breaks this). The big jitter
            # is in approach / retract / idle / loading.
            cut_s=self.nominal_cut_s * r.uniform(0.98, 1.02),
            retract_s=r.uniform(0.5, 1.5),
            idle_s=r.uniform(1.0, 3.0),
            stop_s=r.uniform(2.0, 6.0),
            load=min(1.08, max(0.92, r.gauss(1.0, 0.03))),
            break_at=break_at,
            air_cut=air_cut,
            jam_at=jam_at,
        )

    def _g(self, sd: float) -> float:
        return self._rng.gauss(0.0, sd) if self._noise else 0.0

    def _levels(self, p: CyclePlan, t: float) -> tuple[float, float, bool]:
        """(current, vibration, cutting-load-noise) at t seconds into the cycle."""
        if t < RAMP_S:  # spindle start
            k = t / RAMP_S
            return SPINDLE_OFF_A + k * (SPINDLE_IDLE_A - SPINDLE_OFF_A), IDLE_VIB * k, False
        t -= RAMP_S
        if t < p.approach_s:
            return SPINDLE_IDLE_A, IDLE_VIB, False
        t -= p.approach_s
        if t < p.cut_s:
            frac = t / p.cut_s
            if p.jam_at is not None and frac >= p.jam_at:
                return JAM_A, JAM_VIB, True
            if p.air_cut or (p.break_at is not None and frac >= p.break_at):
                return AIR_CUT_A, IDLE_VIB * 1.5, False
            ramp = min(1.0, t / RAMP_S)  # tool engaging
            a = SPINDLE_IDLE_A + ramp * (CUT_A * p.load - SPINDLE_IDLE_A)
            return a, CUT_VIB * max(0.2, ramp), True
        t -= p.cut_s
        if t < p.retract_s + p.idle_s:
            return SPINDLE_IDLE_A, IDLE_VIB, False
        return SPINDLE_OFF_A, OFF_VIB, False

    def run_cycle(self, p: CyclePlan, on_frame: Callable[[SensorFrame], None]) -> None:
        total = RAMP_S + p.approach_s + p.cut_s + p.retract_s + p.idle_s + p.stop_s
        t = 0.0
        while t < total:
            a, v, loaded = self._levels(p, t)
            a += self._g(CUT_NOISE_A if loaded else LOW_NOISE_A)
            v += self._g(VIB_NOISE)
            if self._noise and loaded and self._rng.random() < SPIKE_PROB:
                a *= 1.5
                v *= 2.0
            on_frame(SensorFrame(self.clock.now(), max(0.0, a), max(0.0, v), 0.5))
            self.clock.advance(DT)
            t += DT
