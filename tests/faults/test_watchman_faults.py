"""Watchman in the full system: STOP faults end SAFE through the watchman, graded
faults alert first and stop only past the hard threshold, and the watchman acts even
when the controller is not running."""

from __future__ import annotations

import pytest

from cell.controller.states import State
from cell.drivers.sim.cell import SimCell
from cell.drivers.sim.sensors import SensorFault
from cell.watchman.watchman import Watchman
from tests.faults.harness import ControllerRunner, System, make_system, new_cell, run_with_faults

S = State


def run_cycle(
    cell: SimCell,
    system: System,
    inject_at: State | None = None,
    fault: SensorFault | None = None,
    delay_s: float = 0.0,
) -> ControllerRunner:
    """Run one cycle; inject `fault` once the controller has been in `inject_at` for delay_s."""
    runner = ControllerRunner(system.ctrl, system.watchman)
    armed = [inject_at is not None]
    armed_since: list[float] = []

    def step() -> bool:
        if armed[0] and system.ctrl.state is inject_at:
            if not armed_since:
                armed_since.append(cell.clock.now())
            if cell.clock.now() - armed_since[0] >= delay_s and fault is not None:
                cell.sensors.inject(fault)
                armed[0] = False
        return runner.step()

    run_with_faults(cell, step, max_s=400)
    return runner


def assert_watchman_safe(cell: SimCell, system: System, kind: str) -> None:
    assert system.ctrl.state is S.SAFE, system.ctrl.last_safe_reason
    assert system.ctrl.last_safe_reason.startswith(f"requested: watchman: {kind}")
    assert system.watchman.stopped
    assert cell.cnc.feed_hold_active()
    assert len(system.safe_alerts) == 1
    assert cell.violations == []


@pytest.mark.parametrize(
    ("fault", "at", "delay_s", "kind"),
    [
        (SensorFault.TOOL_BREAK, S.MACHINING, 4.0, "tool_break"),  # breaks mid-cut
        (SensorFault.JAM, S.MACHINING, 2.0, "overload"),
        (SensorFault.STALE, S.PICK_RAW, 0.0, "stale"),
        (SensorFault.DEAD, S.LOAD, 0.0, "stale"),
    ],
)
def test_stop_faults_end_safe_via_watchman(
    fault: SensorFault, at: State, delay_s: float, kind: str
) -> None:
    cell = new_cell()
    system = make_system(cell)
    run_cycle(cell, system, at, fault, delay_s)
    assert_watchman_safe(cell, system, kind)


def test_tool_already_broken_at_cut_start_stops() -> None:
    cell = new_cell()
    system = make_system(cell)
    assert run_cycle(cell, system).done  # reference cycle
    run_cycle(cell, system, S.CLOSE_DOOR, SensorFault.TOOL_BREAK)
    assert_watchman_safe(cell, system, "tool_break")


def test_dead_sensor_stops_an_idle_cell() -> None:
    cell = new_cell()
    system = make_system(cell)
    cell.sensors.inject(SensorFault.DEAD)
    for _ in range(20):
        system.watchman.tick()
        system.ctrl.step()
        cell.clock.advance(0.1)
    assert_watchman_safe(cell, system, "stale")


def test_chip_buildup_short_cut_alerts_but_finishes() -> None:
    cell = new_cell(cycle_s=10.0)
    system = make_system(cell)
    runner = run_cycle(cell, system, S.CLOSE_DOOR, SensorFault.CHIP_BUILDUP)
    assert runner.done, system.ctrl.last_safe_reason
    assert [a.reason.split(":")[1].strip() for a in system.watchman_alerts] == ["chip_buildup"]
    assert system.safe_alerts == []


def test_chip_buildup_long_cut_escalates_to_stop() -> None:
    cell = new_cell(cycle_s=30.0)
    system = make_system(cell)
    run_cycle(cell, system, S.CLOSE_DOOR, SensorFault.CHIP_BUILDUP)
    assert any("chip_buildup" in a.reason for a in system.watchman_alerts)  # alerted first
    assert_watchman_safe(cell, system, "chip_buildup")


def test_tool_wear_alerts_for_cycles_then_stops() -> None:
    cell = new_cell()
    system = make_system(cell)
    cell.sensors.inject(SensorFault.TOOL_WEAR)
    alerts_per_cycle = []
    for _ in range(12):
        before = len(system.watchman_alerts)
        runner = run_cycle(cell, system)
        alerts_per_cycle.append(len(system.watchman_alerts) - before)
        if runner.stopped:
            break
    assert_watchman_safe(cell, system, "tool_wear")
    assert sum(alerts_per_cycle) >= 1  # the owner was warned before the stop
    assert alerts_per_cycle[0] == 0  # the reference cycle never alerts


def test_alerts_are_sent_once_per_kind_per_cut() -> None:
    cell = new_cell(cycle_s=12.0)
    system = make_system(cell)
    run_cycle(cell, system, S.CLOSE_DOOR, SensorFault.CHIP_BUILDUP)
    assert len(system.watchman_alerts) == 1


def test_watchman_holds_the_machine_without_the_controller() -> None:
    """Independence: nobody calls ctrl.step(); the watchman still feed-holds at once."""
    cell = new_cell(cycle_s=60.0)
    requests: list[str] = []
    alerter = make_system(cell).alerter
    watchman = Watchman(cell.cfg, cell.clock, cell.sensors, cell.cnc, requests.append, alerter)
    cell.cnc.clamp()
    cell.clock.advance(1)
    cell.cnc.cycle_start()
    for _ in range(40):  # 4 s of cutting
        watchman.tick()
        cell.clock.advance(0.1)
    cell.sensors.inject(SensorFault.TOOL_BREAK)
    for _ in range(10):
        watchman.tick()
        cell.clock.advance(0.1)
    assert cell.cnc.feed_hold_active()
    assert len(requests) == 1 and requests[0].startswith("watchman: tool_break")


def test_watchman_error_is_a_stop(monkeypatch: pytest.MonkeyPatch) -> None:
    cell = new_cell()
    system = make_system(cell)

    def broken_read() -> None:
        raise OSError("sensor bus")

    monkeypatch.setattr(cell.sensors, "read", broken_read)
    system.watchman.tick()
    system.ctrl.step()
    assert_watchman_safe(cell, system, "stale")
    assert "watchman error: OSError" in system.ctrl.last_safe_reason
