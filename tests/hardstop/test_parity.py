"""C (node firmware) and Python reference must make identical decisions."""

from __future__ import annotations

import ctypes
import random

import numpy as np
import pytest

from cell.watchman.hardstop import HardStop, HardStopConfig, rms
from tests.hardstop.cbind import CHardStop, c_rms
from tests.hardstop.streams import Stream, edge_streams, fuzz_streams, sim_streams


def run(impl: HardStop | CHardStop, s: Stream) -> list[int]:
    return [
        impl.step(t, a, v, clip, hint)
        for t, a, v, clip, hint in zip(s.t_ms, s.current, s.vib, s.clipped, s.hint, strict=True)
    ]


def all_streams(cfg: HardStopConfig) -> list[Stream]:
    return sim_streams() + edge_streams(cfg) + fuzz_streams()


STEPS = {"total": 0}


def test_parity_on_every_golden_stream(dll: ctypes.CDLL, hcfg: HardStopConfig) -> None:
    mismatches = []
    for s in all_streams(hcfg):
        py, c = run(HardStop(hcfg), s), run(CHardStop(dll, hcfg), s)
        STEPS["total"] += len(s)
        for i, (p, q) in enumerate(zip(py, c, strict=True)):
            if p != q:
                mismatches.append((s.name, i, s.t_ms[i], s.current[i], s.vib[i], p, q))
                break
    assert mismatches == [], mismatches[:5]
    assert STEPS["total"] > 50_000


def test_cut_mean_matches(dll: ctypes.CDLL, hcfg: HardStopConfig) -> None:
    for s in sim_streams()[:6]:
        p, c = HardStop(hcfg), CHardStop(dll, hcfg)
        for t, a, v, clip, hint in zip(s.t_ms, s.current, s.vib, s.clipped, s.hint, strict=True):
            p.step(t, a, v, clip, hint)
            c.step(t, a, v, clip, hint)
            assert p.cut_mean() == c.cut_mean()


def test_rms_bit_identical(dll: ctypes.CDLL) -> None:
    rng = random.Random(1)
    cases = [[], [0], [32767] * 5, [-32768, 32767, 0], list(range(-100, 100))]
    # Summation order matters here: past 2**24 float32 can't represent +1, so a
    # sequential sum drops the trailing ones and a pairwise sum keeps them.
    cases.append([32767] * 520 + [1] * 1000)
    for n in (1, 7, 64, 860, 3200):
        cases.append([rng.randint(-32768, 32767) for _ in range(n)])
        cases.append([int(8000 * np.sin(2 * np.pi * 50 * k / 860)) + 120 for k in range(n)])
    for samples in cases:
        for scale in (1.0, 0.000125, 3.3 / 32768):
            assert rms(samples, scale) == c_rms(dll, samples, scale), (len(samples), scale)


@pytest.mark.parametrize("hint", [-1, 1])
def test_events_actually_fire_in_the_streams(hcfg: HardStopConfig, hint: int) -> None:
    """Guard against a vacuous parity pass: every event type occurs somewhere."""
    seen = 0
    for s in all_streams(hcfg):
        for m in run(HardStop(hcfg), s):
            seen |= m
    assert seen == 0b11111
