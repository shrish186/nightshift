"""Rule-by-rule behaviour, run against BOTH implementations (C and Python), so a rule
that is wrong identically in both still fails here."""

from __future__ import annotations

import ctypes
from collections.abc import Callable

import pytest

from cell.watchman.hardstop import (
    HS_CUT_END,
    HS_CUT_START,
    HS_OVERLOAD,
    HS_SENSOR_FAULT,
    HS_TOOL_BREAK,
    HardStop,
    HardStopConfig,
)
from tests.hardstop.cbind import CHardStop
from tests.hardstop.streams import WRAP, Stream, down, edge_streams, up

Impl = HardStop | CHardStop


@pytest.fixture(params=["python", "c"])
def make(request: pytest.FixtureRequest, dll: ctypes.CDLL) -> Callable[[HardStopConfig], Impl]:
    if request.param == "python":
        return lambda cfg: HardStop(cfg)
    return lambda cfg: CHardStop(dll, cfg)


def feed(impl: Impl, s: Stream) -> list[int]:
    return [impl.step(t, a, v, c, h) for t, a, v, c, h in
            zip(s.t_ms, s.current, s.vib, s.clipped, s.hint, strict=True)]  # fmt: skip


def edge(cfg: HardStopConfig, name: str) -> Stream:
    return next(s for s in edge_streams(cfg) if s.name == name)


def first(masks: list[int], bit: int) -> int | None:
    return next((i for i, m in enumerate(masks) if m & bit), None)


def any_bit(masks: list[int], bit: int) -> bool:
    return any(m & bit for m in masks)


def test_cut_starts_only_strictly_above_threshold(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-cut-on-threshold")
    m = feed(make(hcfg), s)
    i = first(m, HS_CUT_START)
    assert i is not None and s.current[i] == up(hcfg.cut_on_a)  # never at the threshold
    assert s.t_ms[i] - s.t_ms[20] == hcfg.cut_on_ms  # after exactly the confirm window


@pytest.mark.parametrize(("label", "starts"), [("short", False), ("exact", True)])
def test_cut_on_confirm_window_boundary(make, hcfg, label: str, starts: bool) -> None:  # type: ignore[no-untyped-def]
    assert (
        any_bit(feed(make(hcfg), edge(hcfg, f"edge-cut-on-confirm-{label}")), HS_CUT_START)
        is starts
    )


def test_cut_off_hysteresis(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-cut-off")
    m = feed(make(hcfg), s)
    end = first(m, HS_CUT_END)
    assert end is not None and s.current[end] == down(hcfg.cut_off_a)  # not at cut_off_a
    start_low = next(i for i, a in enumerate(s.current) if a == down(hcfg.cut_off_a))
    assert s.t_ms[end] - s.t_ms[start_low] == hcfg.cut_off_ms


def test_overload_strictly_above_and_held(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-overload-current")
    m = feed(make(hcfg), s)
    at_limit = [i for i, a in enumerate(s.current) if a == hcfg.overload_a]
    assert not any(m[i] & HS_OVERLOAD for i in at_limit)
    hits = [i for i, x in enumerate(m) if x & HS_OVERLOAD]
    assert len(hits) == 1  # the confirm-1 ms burst never fires; the exact-confirm one does
    assert s.t_ms[hits[0]] - s.t_ms[hits[0] - 1] == hcfg.overload_confirm_ms


def test_overload_on_vibration(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-overload-vibration")
    m = feed(make(hcfg), s)
    i = first(m, HS_OVERLOAD)
    assert i is not None and s.vib[i] == up(hcfg.vib_max_g)


def test_tool_break_never_at_the_limit(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    assert not any_bit(feed(make(hcfg), edge(hcfg, "edge-tool-break-at-limit")), HS_TOOL_BREAK)


def test_tool_break_one_ulp_below_fires_after_exact_confirm(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-tool-break-below")
    m = feed(make(hcfg), s)
    i = first(m, HS_TOOL_BREAK)
    first_below = next(k for k, a in enumerate(s.current) if a < 12.0)
    assert s.current[first_below] == down(6.0)  # 1 ULP below 0.5 x 12.0
    assert i is not None and s.t_ms[i] - s.t_ms[first_below] == hcfg.break_confirm_ms


def test_no_tool_break_during_settle_or_before_min_samples(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    m = feed(make(hcfg), edge(hcfg, "edge-break-during-settle"))
    assert not any_bit(m, HS_TOOL_BREAK)


def test_cut_ending_clears_pending_break(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    m = feed(make(hcfg), edge(hcfg, "edge-cut-ends-mid-confirm"))
    assert any_bit(m, HS_CUT_END) and not any_bit(m, HS_TOOL_BREAK)


def test_invalid_inputs_are_sensor_faults_never_silent(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-invalid-inputs")
    m = feed(make(hcfg), s)
    assert all(x == HS_SENSOR_FAULT for x in m[-7:-1])  # each bad value flagged at once
    assert not m[-1] & HS_SENSOR_FAULT  # a good value afterwards is fine


def test_flatline_and_clipping_after_window(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-sensor-faults")
    m = feed(make(hcfg), s)
    flat = [i for i, v in enumerate(s.vib) if v == down(hcfg.vib_floor_g)]
    hit = [i for i in flat if m[i] & HS_SENSOR_FAULT]
    assert hit and s.t_ms[hit[0]] - s.t_ms[flat[0]] == hcfg.sensor_fault_ms
    clip = [i for i, c in enumerate(s.clipped) if c]
    hit = [i for i in clip if m[i] & HS_SENSOR_FAULT]
    assert hit and s.t_ms[hit[0]] - s.t_ms[clip[0]] == hcfg.sensor_fault_ms


def test_wraparound_does_not_break_timing(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    s = edge(hcfg, "edge-wraparound")
    m = feed(make(hcfg), s)
    i = first(m, HS_TOOL_BREAK)
    assert i is not None
    assert any(t < 10_000 for t in s.t_ms) and any(t > WRAP - 10_000 for t in s.t_ms)
    j = next(k for k, a in enumerate(s.current) if a == 1.0)
    assert (s.t_ms[i] - s.t_ms[j]) % WRAP == hcfg.break_confirm_ms


def test_cnc_hint_overrides_detection(make, hcfg) -> None:  # type: ignore[no-untyped-def]
    impl = make(hcfg)
    assert impl.step(0, 0.5, 0.05, False, 1) & HS_CUT_START  # cutting at once, low current
    assert impl.step(100, 30.0, 0.8, False, 0) & HS_CUT_END
