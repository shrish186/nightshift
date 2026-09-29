"""False stops: with the fault injector disabled, every cycle must complete.

A cell that stops for no reason gets switched off by the shop, so the false-stop
rate is tracked as seriously as unsafe events. The rate is reported at the end of
every pytest run (see tests/conftest.py) and written to data/reports/.
"""

from __future__ import annotations

import random

from hypothesis import HealthCheck, given, settings

from tests.faults.harness import (
    FALSE_STOP_REPORT,
    CautiousScript,
    NoFaultVariation,
    cell_for,
    no_fault_variations,
    run_cycles,
)

RUNNER_NAME = "CautiousScript"  # becomes the real controller in step 3
REPORT_RUNS = 200
REPORT_CYCLES_PER_RUN = 5


@settings(max_examples=200, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(v=no_fault_variations())
def test_no_faults_means_no_stops(v: NoFaultVariation) -> None:
    cell = cell_for(v)
    result = run_cycles(cell, CautiousScript, n_cycles=3)
    assert result.stops == [], (v, result.stops)
    assert result.completed == 3
    assert cell.violations == []


def test_false_stop_rate_report() -> None:
    """Fixed-seed batch so the reported rate is comparable run to run."""
    rng = random.Random(20260929)
    cycles = stops = 0
    reasons: dict[str, int] = {}
    for i in range(REPORT_RUNS):
        v = NoFaultVariation(
            seed=i,
            no_release_sensor=rng.random() < 0.5,
            door_scale=rng.uniform(0.8, 1.1),
            clamp_scale=rng.uniform(0.8, 1.1),
            robot_speed_scale=rng.uniform(0.8, 1.2),
            cycle_s=rng.uniform(5.0, 20.0),
        )
        result = run_cycles(cell_for(v), CautiousScript, REPORT_CYCLES_PER_RUN)
        cycles += result.completed + len(result.stops)
        stops += len(result.stops)
        for r in result.stops:
            reasons[r] = reasons.get(r, 0) + 1
    FALSE_STOP_REPORT[RUNNER_NAME] = {
        "runs": REPORT_RUNS,
        "cycles": cycles,
        "stops": stops,
        "stop_rate": stops / cycles if cycles else 0.0,
        "reasons": reasons,
    }
    assert stops == 0, reasons
