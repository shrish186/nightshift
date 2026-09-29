"""False stops: with the fault injector disabled, every cycle must complete.

A cell that stops for no reason gets switched off by the shop, so the false-stop
rate is tracked as seriously as unsafe events. Controller and watchman stops are
counted separately, and watchman alerts (which don't stop the cell) are counted too.
The rates are reported at the end of every pytest run (see tests/conftest.py) and
written to data/reports/. The sim's realistic sensor noise is on throughout.
"""

from __future__ import annotations

import random

from hypothesis import HealthCheck, given, settings

from tests.faults.harness import (
    FALSE_STOP_REPORT,
    NoFaultVariation,
    cell_for,
    no_fault_variations,
    run_cycles,
    stop_source,
    system_runners,
)

REPORT_RUNS = 200
REPORT_CYCLES_PER_RUN = 5


@settings(max_examples=200, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(v=no_fault_variations())
def test_no_faults_means_no_stops(v: NoFaultVariation) -> None:
    cell = cell_for(v)
    new_runner, _ = system_runners()
    result = run_cycles(cell, new_runner, n_cycles=3)
    assert result.stops == [], (v, result.stops)
    assert result.completed == 3
    assert cell.violations == []


def test_false_stop_rate_report() -> None:
    """Fixed-seed batch so the reported rates are comparable run to run."""
    rng = random.Random(20260929)
    cycles = 0
    stops: dict[str, dict[str, int]] = {"CellController": {}, "Watchman": {}}
    watchman_alerts: dict[str, int] = {}
    for i in range(REPORT_RUNS):
        v = NoFaultVariation(
            seed=i,
            no_release_sensor=rng.random() < 0.5,
            door_scale=rng.uniform(0.8, 1.1),
            clamp_scale=rng.uniform(0.8, 1.1),
            robot_speed_scale=rng.uniform(0.8, 1.2),
            cycle_s=rng.uniform(5.0, 20.0),
        )
        cell = cell_for(v)
        new_runner, systems = system_runners()
        result = run_cycles(cell, new_runner, REPORT_CYCLES_PER_RUN)
        cycles += result.completed + len(result.stops)
        for r in result.stops:
            bucket = stops[stop_source(r)]
            bucket[r] = bucket.get(r, 0) + 1
        for system in systems.values():
            for a in system.watchman_alerts:
                watchman_alerts[a.reason] = watchman_alerts.get(a.reason, 0) + 1

    for source, reasons in stops.items():
        n = sum(reasons.values())
        FALSE_STOP_REPORT[source] = {
            "runs": REPORT_RUNS,
            "cycles": cycles,
            "stops": n,
            "stop_rate": n / cycles if cycles else 0.0,
            "reasons": reasons,
        }
    n_alerts = sum(watchman_alerts.values())
    FALSE_STOP_REPORT["Watchman"]["alerts"] = n_alerts
    FALSE_STOP_REPORT["Watchman"]["alert_rate"] = n_alerts / cycles if cycles else 0.0
    FALSE_STOP_REPORT["Watchman"]["alert_reasons"] = watchman_alerts
    assert stops == {"CellController": {}, "Watchman": {}}, stops
