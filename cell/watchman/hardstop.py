"""Hard-stop rules: the Python reference for node/lib/hardstop (C).

SAFETY-RELEVANT: changes need plan mode and human review (see CLAUDE.md).

This must make IDENTICAL decisions to the C code on the same inputs: change both
together and keep tests/hardstop/ (the parity suite) green. To match the ESP32 bit for
bit, all arithmetic is numpy float32, sums are sequential (never np.sum, which adds
pairwise), and time is uint32 milliseconds with wraparound.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cell.config import WatchmanConfig

f32 = np.float32
_U32 = 0xFFFFFFFF

HS_CUT_START = 1 << 0
HS_CUT_END = 1 << 1
HS_OVERLOAD = 1 << 2
HS_TOOL_BREAK = 1 << 3
HS_SENSOR_FAULT = 1 << 4

MIN_SETTLED = 5


@dataclass(frozen=True)
class HardStopConfig:
    cut_on_a: float
    cut_off_a: float
    cut_on_ms: int
    cut_off_ms: int
    settle_ms: int
    break_ratio: float
    break_confirm_ms: int
    min_settled: int
    overload_a: float
    vib_max_g: float
    overload_confirm_ms: int
    vib_floor_g: float
    sensor_fault_ms: int

    @classmethod
    def from_watchman(cls, w: WatchmanConfig) -> HardStopConfig:
        def ms(s: float) -> int:
            return round(s * 1000)

        return cls(
            cut_on_a=w.cut_on_current_a,
            cut_off_a=w.cut_off_current_a,
            cut_on_ms=ms(w.cut_on_confirm_s),
            cut_off_ms=ms(w.cut_off_confirm_s),
            settle_ms=ms(w.cut_settle_s),
            break_ratio=w.tool_break_current_ratio,
            break_confirm_ms=ms(w.tool_break_confirm_s),
            min_settled=MIN_SETTLED,
            overload_a=w.overload_current_a,
            vib_max_g=w.vibration_rms_max_g,
            overload_confirm_ms=ms(w.overload_confirm_s),
            vib_floor_g=w.vib_floor_g,
            sensor_fault_ms=ms(w.sensor_fault_confirm_s),
        )


def _elapsed(now: int, since: int) -> int:
    return (now - since) & _U32


class _Held:
    """A condition held for at least `window` ms (C: held())."""

    __slots__ = ("pending", "since")

    def __init__(self) -> None:
        self.pending = False
        self.since = 0

    def __call__(self, cond: bool, t_ms: int, window: int) -> bool:
        if not cond:
            self.pending = False
            return False
        if not self.pending:
            self.pending = True
            self.since = t_ms
        return _elapsed(t_ms, self.since) >= window


class HardStop:
    def __init__(self, cfg: HardStopConfig) -> None:
        self.cfg = cfg
        self._cut_on_a = f32(cfg.cut_on_a)
        self._cut_off_a = f32(cfg.cut_off_a)
        self._break_ratio = f32(cfg.break_ratio)
        self._overload_a = f32(cfg.overload_a)
        self._vib_max_g = f32(cfg.vib_max_g)
        self._vib_floor_g = f32(cfg.vib_floor_g)
        self.cutting = False
        self._on = _Held()
        self._off = _Held()
        self._break = _Held()
        self._over = _Held()
        self._fault = _Held()
        self._cut_start = 0
        self._sum = f32(0.0)
        self._n = 0

    def cut_mean(self) -> float:
        return 0.0 if self._n == 0 else float(self._sum / f32(self._n))

    def _new_cut(self, t_ms: int) -> None:
        self._cut_start = t_ms
        self._sum = f32(0.0)
        self._n = 0
        self._break.pending = False
        self._over.pending = False

    def step(
        self, t_ms: int, current_a: float, vib_g: float, clipped: bool, cutting_hint: int
    ) -> int:
        c = self.cfg
        t_ms &= _U32
        a, v = f32(current_a), f32(vib_g)
        ev = 0

        # Invalid input never passes silently.
        if not (np.isfinite(a) and np.isfinite(v)) or a < f32(0.0) or v < f32(0.0):
            return HS_SENSOR_FAULT

        # --- cut detection ---
        was = self.cutting
        if cutting_hint >= 0:
            now_cut = bool(cutting_hint)
            self._on.pending = False
            self._off.pending = False
        elif not was:
            self._off.pending = False
            now_cut = self._on(bool(a > self._cut_on_a), t_ms, c.cut_on_ms)
        else:
            self._on.pending = False
            now_cut = not self._off(bool(a < self._cut_off_a), t_ms, c.cut_off_ms)
        if now_cut and not was:
            ev |= HS_CUT_START
            self._new_cut(t_ms)
        if not now_cut and was:
            ev |= HS_CUT_END
        self.cutting = now_cut

        # --- sensor fault (ADC clipping any time; vibration flat while cutting) ---
        fault = bool(clipped) or (now_cut and bool(v < self._vib_floor_g))
        if self._fault(fault, t_ms, c.sensor_fault_ms):
            ev |= HS_SENSOR_FAULT

        if not now_cut:
            self._over.pending = False
            self._break.pending = False
            return ev

        # --- overload: current or vibration above limit, held ---
        over = bool(a > self._overload_a) or bool(v > self._vib_max_g)
        if self._over(over, t_ms, c.overload_confirm_ms):
            ev |= HS_OVERLOAD

        if _elapsed(t_ms, self._cut_start) < c.settle_ms:
            return ev

        # --- tool break: current collapses vs. this cut's settled mean, held ---
        below = False
        if self._n >= c.min_settled:
            mean = f32(self._sum / f32(self._n))
            limit = f32(self._break_ratio * mean)
            below = bool(a < limit)
        if self._break(below, t_ms, c.break_confirm_ms):
            ev |= HS_TOOL_BREAK
        if not below and not over:
            self._sum = f32(self._sum + a)
            self._n += 1
        return ev


def rms(samples: list[int], scale: float) -> tuple[float, int]:
    """Python reference for hs_rms(): (rms * scale, clipped count). Sequential float32."""
    n = len(samples)
    if n == 0:
        return 0.0, 0
    total = f32(0.0)
    clipped = 0
    for s in samples:
        total = f32(total + f32(s))
        if s in (32767, -32768):
            clipped += 1
    mean = f32(total / f32(n))
    acc = f32(0.0)
    for s in samples:
        d = f32(f32(s) - mean)
        acc = f32(acc + f32(d * d))
    return float(f32(np.sqrt(f32(acc / f32(n))) * f32(scale))), clipped
