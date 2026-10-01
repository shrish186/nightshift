"""Standalone machine (the December pilot setup): no false tool breaks on normal cycles,
real breaks caught, air cuts caught. Founder-found bug (2026-10): every normal cut end
used to fire a tool break in standalone mode."""

from __future__ import annotations

import pytest

from tests.faults.harness import FALSE_STOP_REPORT
from tests.faults.standalone import make_standalone

N_CYCLES = 1000


def test_no_false_tool_breaks_over_1000_jittered_cycles() -> None:
    s = make_standalone(seed=7)
    stops: list[str] = []
    for _ in range(N_CYCLES):
        stops += s.run(s.machine.plan())
        if s.watchman.stopped:  # a (false) stop latches: count it and keep going
            s.watchman.reset()
            s.relay.held = False
    tool_alerts = [a for a in s.alerts if "tool_break" in a]
    FALSE_STOP_REPORT["Watchman (standalone)"] = {
        "runs": 1,
        "cycles": N_CYCLES,
        "stops": len(stops),
        "stop_rate": len(stops) / N_CYCLES,
        "reasons": {r: stops.count(r) for r in set(stops)},
        "alerts": len(s.alerts),
        "alert_rate": len(s.alerts) / N_CYCLES,
        "alert_reasons": {r: s.alerts.count(r) for r in set(s.alerts)},
    }
    assert stops == [], stops[:3]
    assert tool_alerts == []
    assert s.watchman.recording_progress is None


@pytest.mark.parametrize("break_at", [0.1, 0.3, 0.5, 0.7])
def test_real_break_mid_cut_is_caught(break_at: float) -> None:
    s = make_standalone(seed=3)
    stops = s.run(s.machine.plan(break_at=break_at))
    assert len(stops) == 1 and "tool_break: load ended after" in stops[0], stops
    assert s.relay.held  # auto-stop path: the node relay holds the machine


def test_break_near_the_end_is_missed_then_caught_next_cycle_as_air_cut() -> None:
    """Documented limit: a break in the last ~20% of the cut looks like a normal end."""
    s = make_standalone(seed=4)
    assert s.run(s.machine.plan(break_at=0.9)) == []
    stops = s.run(s.machine.plan(air_cut=True))  # the broken tool, next cycle
    assert len(stops) == 1 and "air cut" in stops[0]


def test_tool_broken_from_the_start_is_an_air_cut() -> None:
    s = make_standalone(seed=5)
    stops = s.run(s.machine.plan(air_cut=True))
    assert len(stops) == 1 and "air cut: no load" in stops[0]


def test_jam_is_an_overload_from_the_node_rule() -> None:
    s = make_standalone(seed=6)
    stops = s.run(s.machine.plan(jam_at=0.4))
    assert len(stops) == 1 and "overload" in stops[0]


def test_without_a_reference_load_ends_are_never_judged() -> None:
    s = make_standalone(seed=8, record=False)
    assert s.watchman.prepare("P1", supervised=True) is None
    assert s.run(s.machine.plan(break_at=0.3)) == []  # no reference: no judgement


def test_reference_records_load_timing() -> None:
    s = make_standalone(seed=9)
    ref = s.watchman.reference_for("P1")
    assert ref is not None and ref.min_load_s is not None and ref.max_load_start_s is not None
    assert 11.0 < ref.min_load_s < 12.5  # ~12 s programmed cut
    assert 1.0 < ref.max_load_start_s < 4.0  # ramp + approach
