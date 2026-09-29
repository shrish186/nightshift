"""Watchman detector unit tests on synthetic sample streams (10 Hz, like the harness)."""

from __future__ import annotations

import random
from collections.abc import Callable

import pytest

from cell.config import WatchmanConfig, load_cell_config
from cell.drivers.sensors import SensorFrame
from cell.watchman.detector import Detector, Finding, FindingKind, Severity
from tests.conftest import SIM_CELL

K, SEV = FindingKind, Severity
DT = 0.1
CUT_A = 12.0


@pytest.fixture
def wcfg() -> WatchmanConfig:
    return load_cell_config(SIM_CELL).watchman


class Feed:
    """Drives a Detector with synthetic frames and collects every finding."""

    def __init__(self, cfg: WatchmanConfig, seed: int = 0) -> None:
        self.det = Detector(cfg, start=0.0)
        self.t = 0.0
        self.rng = random.Random(seed)
        self.findings: list[Finding] = []

    def run(
        self,
        seconds: float,
        current: Callable[[float], float] = lambda t: CUT_A,
        vib: Callable[[float], float] = lambda t: 0.8,
        cutting: bool = True,
        noise: float = 0.3,
        frame_ts: Callable[[float], float | None] = lambda t: t,
    ) -> list[Finding]:
        new: list[Finding] = []
        t0 = self.t
        while self.t - t0 < seconds - 1e-9:
            ts = frame_ts(self.t)
            frame = None
            if ts is not None:
                a = current(self.t - t0) if cutting else 1.0
                frame = SensorFrame(ts, a + self.rng.gauss(0, noise), vib(self.t - t0), 0.5)
            new += self.det.update(frame, self.t, cutting)
            self.t = round(self.t + DT, 6)
        self.findings += new
        return new

    def idle(self, seconds: float = 2.0) -> list[Finding]:
        return self.run(seconds, cutting=False)


def kinds(fs: list[Finding]) -> set[tuple[FindingKind, Severity]]:
    return {(f.kind, f.severity) for f in fs}


# --- no false findings ---


def test_noise_alone_produces_nothing(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg, seed=3)
    for _ in range(20):
        f.idle()
        f.run(10, current=lambda t: CUT_A * (1 + random.Random(int(t)).gauss(0, 0.01)))
    f.idle()
    assert f.findings == []


def test_ramp_up_is_ignored(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    f.idle()
    assert f.run(5, current=lambda t: CUT_A * min(1.0, t / 0.5)) == []


def test_single_sample_spikes_and_dips_are_ignored(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    f.idle()
    f.run(3)
    assert f.run(1, current=lambda t: 40.0 if t < DT else CUT_A) == []  # one-sample spike
    assert f.run(1, current=lambda t: 1.0 if t < DT else CUT_A) == []  # one-sample dip
    assert f.run(1, vib=lambda t: 9.0 if t < DT else 0.8) == []


# --- STOP detectors, both sides of the confirm window ---


def test_tool_break_stops_after_confirm(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    f.idle()
    f.run(3)
    short = wcfg.tool_break_confirm_s - DT
    assert f.run(short, current=lambda t: CUT_A * 0.25) == []
    f.run(2)  # recovers: not a break
    out = f.run(1, current=lambda t: CUT_A * 0.25)
    assert (K.TOOL_BREAK, SEV.STOP) in kinds(out)


@pytest.mark.parametrize("channel", ["current", "vibration"])
def test_overload_stops_after_confirm(wcfg: WatchmanConfig, channel: str) -> None:
    f = Feed(wcfg)
    f.idle()
    f.run(3)
    over_a = (lambda t: 30.0) if channel == "current" else (lambda t: CUT_A)
    over_v = (lambda t: 5.0) if channel == "vibration" else (lambda t: 0.8)
    short = wcfg.overload_confirm_s - DT
    assert f.run(short, current=over_a, vib=over_v) == []
    f.run(1)
    assert (K.OVERLOAD, SEV.STOP) in kinds(f.run(1, current=over_a, vib=over_v))


def test_stale_frame_stops(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    f.idle()
    frozen = f.t
    out = f.run(wcfg.stale_after_s - DT, cutting=False, frame_ts=lambda t: frozen)
    assert out == []
    out = f.run(2 * DT, cutting=False, frame_ts=lambda t: frozen)
    assert (K.STALE, SEV.STOP) in kinds(out)


def test_no_frames_at_all_stops_after_grace(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    assert f.run(wcfg.stale_after_s - DT, cutting=False, frame_ts=lambda t: None) == []
    assert (K.STALE, SEV.STOP) in kinds(f.run(2 * DT, cutting=False, frame_ts=lambda t: None))


def test_stale_stops_while_idle_too(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    f.idle(5)
    assert (K.STALE, SEV.STOP) in kinds(f.run(1, cutting=False, frame_ts=lambda t: None))


# --- graded: chip buildup (within a cycle) ---


def _chip(wcfg: WatchmanConfig, final_ratio: float, seconds: float = 12.0) -> list[Finding]:
    f = Feed(wcfg, seed=1)
    f.idle()
    settle = wcfg.cut_settle_s

    def ramp(t: float) -> float:
        k = max(0.0, t - settle) / (seconds - settle)
        return CUT_A * (1 + (final_ratio - 1) * k)

    return f.run(seconds, current=ramp)


def test_chip_below_alert_is_quiet(wcfg: WatchmanConfig) -> None:
    assert _chip(wcfg, 1.1) == []


def test_chip_between_alert_and_stop_only_alerts(wcfg: WatchmanConfig) -> None:
    out = kinds(_chip(wcfg, 1.35))
    assert (K.CHIP_BUILDUP, SEV.ALERT) in out
    assert (K.CHIP_BUILDUP, SEV.STOP) not in out


def test_chip_past_hard_threshold_stops(wcfg: WatchmanConfig) -> None:
    out = kinds(_chip(wcfg, 1.9, seconds=16))
    assert (K.CHIP_BUILDUP, SEV.ALERT) in out
    assert (K.CHIP_BUILDUP, SEV.STOP) in out


# --- graded: tool wear (across cycles) ---


def _constant(amps: float) -> Callable[[float], float]:
    return lambda t: amps


def _wear(wcfg: WatchmanConfig, ratios: list[float]) -> list[list[Finding]]:
    f = Feed(wcfg, seed=2)
    per_cycle = []
    for r in [1.0, *ratios]:  # first cycle is the reference
        f.idle()
        found = f.run(8, current=_constant(CUT_A * r))
        found += f.idle(0.5)  # wear is judged when the cycle ends
        per_cycle.append(found)
    return per_cycle[1:]


def test_wear_below_alert_is_quiet(wcfg: WatchmanConfig) -> None:
    assert _wear(wcfg, [1.1, 1.2]) == [[], []]


def test_wear_alerts_then_stops_past_hard_threshold(wcfg: WatchmanConfig) -> None:
    c1, c2 = _wear(wcfg, [1.3, 1.6])
    assert kinds(c1) == {(K.TOOL_WEAR, SEV.ALERT)}
    assert (K.TOOL_WEAR, SEV.STOP) in kinds(c2)


def test_wear_does_not_trip_chip_detector(wcfg: WatchmanConfig) -> None:
    """A uniformly higher load across a whole cycle is wear, not chip buildup."""
    for c in _wear(wcfg, [1.3, 1.45]):
        assert all(f.kind is K.TOOL_WEAR for f in c)
