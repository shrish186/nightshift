"""Prove the unsafe-event log (the watchdog) catches broken logic, and stays empty
for correct logic under random faults."""

from __future__ import annotations

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from cell.drivers.sim.cell import SimCell
from cell.drivers.sim.cnc import CncFault
from tests.faults.harness import (
    CautiousScript,
    FaultEvent,
    fault_schedules,
    new_cell,
    run_with_faults,
)


def _run(
    cell: SimCell, breaks: frozenset[str] = frozenset(), faults: list[FaultEvent] | None = None
) -> CautiousScript:
    script = CautiousScript(cell, breaks)
    run_with_faults(cell, script.step, faults or [])
    return script


# --- sanity: correct logic, no faults ---


@pytest.mark.parametrize("no_release_sensor", [False, True])
def test_cautious_cycle_completes_cleanly(no_release_sensor: bool) -> None:
    cell = new_cell(no_release_sensor)
    script = _run(cell)
    assert script.done, script.stop_reason
    assert cell.violations == []
    assert cell.robot.at_pose() == "home"


# --- deliberately broken logic: the watchdog must catch each one ---

BROKEN = [
    pytest.param(
        "no_door_open_wait",
        [FaultEvent(0, CncFault.DOOR_STUCK)],
        False,
        "door not open",
        id="enter-with-door-half-open",
    ),
    pytest.param(
        "no_plausibility",
        [FaultEvent(0, CncFault.DOOR_SENSOR_SHORT)],
        False,
        "door not open",
        id="enter-trusting-shorted-door-sensor",
    ),
    pytest.param("close_before_retreat", [], False, "door closed on arm", id="door-closes-on-arm"),
    pytest.param(
        "start_before_retreat", [], False, "cycle start with arm in machine", id="start-arm-inside"
    ),
    pytest.param(
        "no_release_check",
        [FaultEvent(0, CncFault.CLAMP_STUCK_ON)],
        False,
        "while still clamped",
        id="pull-stuck-clamp-without-check",
    ),
    pytest.param(
        "no_release_check",
        [],
        True,
        "while still clamped",
        id="pull-without-fallback-on-no-sensor-machine",
    ),
]


@pytest.mark.parametrize(("brk", "faults", "no_release_sensor", "expect"), BROKEN)
def test_watchdog_catches_broken_logic(
    brk: str, faults: list[FaultEvent], no_release_sensor: bool, expect: str
) -> None:
    cell = new_cell(no_release_sensor)
    _run(cell, frozenset({brk}), faults)
    assert any(expect in v for v in cell.violations), cell.violations


@pytest.mark.parametrize(("brk", "faults", "no_release_sensor", "expect"), BROKEN)
def test_same_faults_with_correct_logic_are_safe(
    brk: str, faults: list[FaultEvent], no_release_sensor: bool, expect: str
) -> None:
    # Control: the same situation with the check switched back on produces no violation.
    cell = new_cell(no_release_sensor)
    _run(cell, frozenset(), faults)
    assert cell.violations == []


def test_watchdog_catches_unclamp_during_cycle() -> None:
    cell = new_cell()
    cell.cnc.clamp()
    cell.clock.advance(1)
    cell.cnc.cycle_start()
    assert cell.cnc.cycle_running()
    cell.cnc.unclamp()
    assert any("unclamp during cycle" in v for v in cell.violations)


# --- randomized: correct logic under random fault timing and combinations ---


@settings(max_examples=300, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(
    schedule=fault_schedules(),
    no_release_sensor=st.booleans(),
    seed=st.integers(min_value=0, max_value=2**16),
)
def test_random_faults_never_cause_unsafe_events(
    schedule: list[FaultEvent], no_release_sensor: bool, seed: int
) -> None:
    cell = new_cell(no_release_sensor, seed=seed)
    script = _run(cell, faults=schedule)
    assert cell.violations == [], (schedule, cell.violations)
    # Every run ends either with a completed cycle or stopped (robot stopped + feed hold).
    assert script.done or script.stopped, schedule
    if script.stopped:
        assert cell.cnc.feed_hold_active()
