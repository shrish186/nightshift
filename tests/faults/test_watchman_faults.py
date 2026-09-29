"""Watchman in the full system: STOP faults end SAFE through the watchman, graded
faults alert first and stop only past the hard threshold, and the watchman acts even
when the controller is not running."""

from __future__ import annotations

import pytest

from cell.controller.states import State
from cell.drivers.sim.cell import SimCell
from cell.drivers.sim.sensors import SensorFault
from cell.watchman.watchman import Watchman
from tests.faults.harness import (
    PROGRAM,
    ControllerRunner,
    System,
    make_system,
    new_cell,
    run_with_faults,
)

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


# --- node offline / stale data: "watchman unhealthy", never a stop by itself ---


def _tick(cell: SimCell, system: System, seconds: float) -> None:
    for _ in range(round(seconds / 0.1)):
        system.watchman.tick()
        system.ctrl.step()
        cell.clock.advance(0.1)


@pytest.mark.parametrize("fault", [SensorFault.DEAD, SensorFault.STALE])
def test_dead_sensor_on_an_idle_cell_alerts_and_blocks_unattended_start(
    fault: SensorFault,
) -> None:
    cell = new_cell()
    system = make_system(cell)
    cell.sensors.inject(fault)
    _tick(cell, system, 2)
    assert system.ctrl.state is S.IDLE  # no SAFE: the machine isn't stopped for this
    assert any("stale" in a.reason for a in system.watchman_alerts)
    assert not system.ctrl.start_cycle(PROGRAM)
    assert "no fresh sensor data" in system.ctrl.last_start_refusal
    assert system.ctrl.start_cycle(PROGRAM, supervised=True)  # supervised still allowed


def test_node_drops_out_mid_cut_cycle_finishes_then_next_start_refused() -> None:
    cell = new_cell()
    system = make_system(cell)
    runner = run_cycle(cell, system, S.MACHINING, SensorFault.DEAD, 2.0)
    assert runner.done, system.ctrl.last_safe_reason  # the cut finished, part unloaded
    assert not cell.cnc.feed_hold_active()
    assert cell.violations == []
    assert not system.ctrl.start_cycle(PROGRAM)


def state(system: System) -> State:
    return system.ctrl.state  # (a function, so mypy doesn't narrow across steps)


def test_node_drops_out_before_load_arm_waits_then_continues() -> None:
    cell = new_cell()
    system = make_system(cell)
    ctrl = system.ctrl
    assert ctrl.start_cycle(PROGRAM)
    _tick(cell, system, 0.3)
    assert state(system) is S.PICK_RAW
    cell.sensors.inject(SensorFault.DEAD)  # node drops out while the arm picks the part
    _tick(cell, system, 8)
    assert state(system) is S.WAIT_WATCHMAN
    assert not cell.robot.occupies(cell.cfg.machine_zone_poses)  # never loaded
    assert cell.gripper.has_part()
    cell.sensors.clear(SensorFault.DEAD)  # the node comes back
    for _ in range(3000):
        _tick(cell, system, 0.1)
        if state(system) is S.IDLE:
            break
    assert state(system) is S.IDLE and ctrl.cycles_completed == 1
    assert cell.violations == []


def test_node_stays_offline_past_the_bound_goes_safe_with_arm_outside() -> None:
    cell = new_cell()
    system = make_system(cell)
    ctrl = system.ctrl
    assert ctrl.start_cycle(PROGRAM)
    _tick(cell, system, 0.3)
    cell.sensors.inject(SensorFault.DEAD)
    _tick(cell, system, 8)
    assert state(system) is S.WAIT_WATCHMAN
    _tick(cell, system, cell.cfg.timeouts_s.watchman_wait_s + 2)
    assert state(system) is S.SAFE
    assert "watchman healthy not within" in ctrl.last_safe_reason
    assert not cell.robot.occupies(cell.cfg.machine_zone_poses)
    assert cell.violations == []


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


def test_watchman_error_makes_it_unhealthy(monkeypatch: pytest.MonkeyPatch) -> None:
    cell = new_cell()
    system = make_system(cell)

    def broken_read() -> None:
        raise OSError("sensor bus")

    monkeypatch.setattr(cell.sensors, "read", broken_read)
    _tick(cell, system, 1)
    assert system.ctrl.state is S.IDLE
    assert any("watchman error: OSError" in a.reason for a in system.watchman_alerts)
    assert not system.ctrl.start_cycle(PROGRAM)


# --- reset refused while the watchman is unhealthy ---


def _tool_break_safe() -> tuple[SimCell, System]:
    cell = new_cell()
    system = make_system(cell)
    run_cycle(cell, system, S.MACHINING, SensorFault.TOOL_BREAK, 4.0)
    assert_watchman_safe(cell, system, "tool_break")
    for _ in range(50):  # let the controller finish reacting; arm is outside, door closed
        system.watchman.tick()
        system.ctrl.step()
        cell.clock.advance(0.1)
    cell.cnc.operator_clear()
    return cell, system


def test_reset_refused_while_watchman_stop_is_latched() -> None:
    _, system = _tool_break_safe()
    why = system.ctrl.reset("asha")
    assert why is not None and "watchman stop latched" in why
    system.watchman.reset()  # operator changed the tool and reset the watchman
    assert system.ctrl.reset("asha") is None


def test_reset_refused_while_sensor_data_is_stale() -> None:
    cell, system = _tool_break_safe()
    cell.sensors.inject(SensorFault.DEAD)
    system.watchman.reset()
    why = system.ctrl.reset("asha")
    assert why is not None and "no fresh sensor data" in why
