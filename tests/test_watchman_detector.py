"""Watchman detector unit tests on synthetic sample streams (10 Hz, like the harness)."""

from __future__ import annotations

import random
from collections.abc import Callable

import pytest

from cell.config import WatchmanConfig, load_cell_config
from cell.drivers.sensors import SensorFrame
from cell.watchman.detector import Detector, Finding, FindingKind, Severity
from cell.watchman.reference import CycleStats, Reference
from tests.conftest import SIM_CELL

K, SEV = FindingKind, Severity
PROG = "P1"
DT = 0.1
CUT_A = 12.0


@pytest.fixture
def wcfg() -> WatchmanConfig:
    return load_cell_config(SIM_CELL).watchman


class Feed:
    """Drives a Detector with synthetic frames and collects every finding."""

    def __init__(
        self, cfg: WatchmanConfig, seed: int = 0, reference: Reference | None = None
    ) -> None:
        self.det = Detector(cfg, start=0.0, reference=reference)
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
                # RMS features are never negative (a negative one is a sensor fault)
                amps = max(0.0, a + self.rng.gauss(0, noise))
                frame = SensorFrame(ts, amps, vib(self.t - t0), 0.5)
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


def test_stale_frame_is_flagged(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    f.idle()
    frozen = f.t
    out = f.run(wcfg.stale_after_s - DT, cutting=False, frame_ts=lambda t: frozen)
    assert out == []
    out = f.run(2 * DT, cutting=False, frame_ts=lambda t: frozen)
    assert (K.STALE, SEV.ALERT) in kinds(out)


def test_no_frames_at_all_flagged_after_grace(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    assert f.run(wcfg.stale_after_s - DT, cutting=False, frame_ts=lambda t: None) == []
    assert (K.STALE, SEV.ALERT) in kinds(f.run(2 * DT, cutting=False, frame_ts=lambda t: None))


def test_stale_flagged_while_idle_too(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    f.idle(5)
    assert (K.STALE, SEV.ALERT) in kinds(f.run(1, cutting=False, frame_ts=lambda t: None))


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


def ref(levels: list[float], chips: list[float] | None = None) -> Reference:
    """A confirmed reference with the given per-cycle levels (amps) and chip maxima."""
    chips = chips or [1.0] * len(levels)
    return Reference(
        PROG, "T1", "op", True, "2026-09-29T00:00:00+00:00",
        tuple(CycleStats(lv, ch) for lv, ch in zip(levels, chips, strict=True)),
    )  # fmt: skip


# Wide spread, so the derived limits are capped at the config ratios.
WIDE = ref([CUT_A * 0.9, CUT_A, CUT_A * 1.1, CUT_A * 0.95, CUT_A * 1.05])


def _wear(
    wcfg: WatchmanConfig, ratios: list[float], reference: Reference = WIDE
) -> list[list[Finding]]:
    f = Feed(wcfg, seed=2, reference=reference)
    per_cycle = []
    for r in ratios:
        f.idle()
        found = f.run(8, current=_constant(CUT_A * r))
        found += f.idle(0.5)  # wear is judged when the cycle ends
        per_cycle.append(found)
    return per_cycle


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


def test_tool_broken_before_the_cut_starts_stops(wcfg: WatchmanConfig) -> None:
    """Low load from the first sample means the cut's own mean is low too; the reference
    cycle is what catches it."""
    c1, c2 = _wear(wcfg, [0.6, 0.25])
    assert c1 == []  # lighter cut (e.g. smaller stock) but above the break ratio
    assert (K.TOOL_BREAK, SEV.STOP) in kinds(c2)


def test_without_a_reference_nothing_is_learned_or_judged(wcfg: WatchmanConfig) -> None:
    f = Feed(wcfg)
    for _ in range(5):  # clean cycles: never become a reference by themselves
        f.idle()
        f.run(8)
    f.idle()
    assert f.run(8, current=_constant(CUT_A * 0.25)) + f.idle() == []  # nothing to compare
    assert f.run(8, current=_constant(CUT_A * 1.6)) + f.idle() == []


# --- limits derived from the reference spread ---


def test_tight_reference_gives_tighter_wear_limit_than_config(wcfg: WatchmanConfig) -> None:
    tight = ref([CUT_A * x for x in (0.99, 1.0, 1.01, 1.0, 1.0)])
    limits = tight.limits(wcfg)
    assert limits.wear_alert < wcfg.wear_alert_ratio
    assert limits.wear_alert >= 1 + wcfg.derived_min_margin
    # a 1.18x drift: the fixed 1.25 ratio misses it, the derived limit catches it
    assert _wear(wcfg, [1.18], reference=WIDE) == [[]]
    (c,) = _wear(wcfg, [1.18], reference=tight)
    assert kinds(c) == {(K.TOOL_WEAR, SEV.ALERT)}


def test_noisy_reference_is_capped_at_config_ratio(wcfg: WatchmanConfig) -> None:
    noisy = ref([CUT_A * x for x in (0.7, 1.3, 0.8, 1.2, 1.0)], [1.0, 1.4, 1.0, 1.3, 1.1])
    limits = noisy.limits(wcfg)
    assert limits.wear_alert == wcfg.wear_alert_ratio
    assert limits.chip_alert == wcfg.chip_alert_ratio


def test_zero_spread_still_keeps_the_minimum_margin(wcfg: WatchmanConfig) -> None:
    flat = ref([CUT_A] * 5)
    limits = flat.limits(wcfg)
    assert limits.wear_alert == pytest.approx(1 + wcfg.derived_min_margin)
    assert limits.chip_alert == pytest.approx(1 + wcfg.derived_min_margin)


def test_tight_reference_gives_tighter_chip_limit(wcfg: WatchmanConfig) -> None:
    tight = ref([CUT_A] * 5, [1.0, 1.01, 1.0, 1.02, 1.0])
    assert tight.limits(wcfg).chip_alert < wcfg.chip_alert_ratio
    f = Feed(wcfg, seed=1, reference=tight)
    f.idle()

    def drift(t: float) -> float:  # ends near 1.19x: under the 1.2 config alert
        return CUT_A * (1 + 0.195 * max(0.0, t - wcfg.cut_settle_s) / 11)

    # with no reference the fixed config ratio (1.2) applies, and it misses this drift
    fixed = Feed(wcfg, seed=1)
    fixed.idle()
    assert (K.CHIP_BUILDUP, SEV.ALERT) not in kinds(fixed.run(12, current=drift))

    assert (K.CHIP_BUILDUP, SEV.ALERT) in kinds(f.run(12, current=drift))
