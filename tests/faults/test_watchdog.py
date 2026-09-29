"""Prove the unsafe-event log (the watchdog) catches broken controller logic, and
stays empty for the real controller under random faults.

Broken logic is made by swapping entries in the controller's STATE_TABLE (via
monkeypatch, test-only) to remove a guard or a wait. There is no switch in the
controller itself to turn a check off.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Callable
from dataclasses import dataclass

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from cell.config import CellConfig
from cell.controller.machine import (
    STATE_TABLE,
    CellController,
    Command,
    Ctx,
    Guard,
    Move,
    Step,
    WaitUntil,
)
from cell.controller.states import State
from cell.drivers.robot import RobotStatus
from cell.drivers.sim.cell import SimCell
from cell.drivers.sim.cnc import CncFault
from cell.watchman.watchman import Watchman
from tests.faults.harness import (
    ControllerRunner,
    FaultEvent,
    fault_schedules,
    make_controller,
    make_system,
    new_cell,
    run_with_faults,
)

S = State


class _NoAlerts:
    def alert(self, alert: object) -> None:
        pass


Mutation = Callable[[pytest.MonkeyPatch, CellController], None]


def _set(
    mp: pytest.MonkeyPatch,
    state: State,
    steps: Callable[[CellConfig], tuple[Step, ...]] | None = None,
    guards: tuple[Guard, ...] | None = None,
) -> None:
    spec = STATE_TABLE[state]
    if steps is not None:
        spec = dataclasses.replace(spec, steps=steps)
    if guards is not None:
        spec = dataclasses.replace(spec, guards=guards)
    mp.setitem(STATE_TABLE, state, spec)


def _steps(*steps: Step) -> Callable[[CellConfig], tuple[Step, ...]]:
    return lambda cfg: steps


def _door_open_only(c: Ctx) -> str | None:  # a weaker guard: trusts door_open alone
    return None if c.cnc.door_open() else "door not open"


def m_no_inside_guard(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    # Both layers off: the LOAD guard and the command-aware plausibility check (rule c).
    m_plausibility_off(mp, ctrl)
    _set(mp, S.LOAD, guards=())


def m_no_seat_checks(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    clamp_steps = STATE_TABLE[S.CLAMP].steps(ctrl._cfg)
    _set(
        mp,
        S.CLAMP,
        steps=_steps(
            *(
                s
                for s in clamp_steps
                if not isinstance(s, WaitUntil) or s.label != "part seated in fixture"
            )
        ),
    )
    _set(mp, S.MACHINING, guards=(_door_closed_and_clamped,))


def _door_closed_and_clamped(c: Ctx) -> str | None:  # machine_ready_to_cut minus the seat check
    return None if c.cnc.door_closed() and c.cnc.clamped() else "not ready"


def m_no_gripper_open_check(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    steps = STATE_TABLE[S.ENTER_UNLOAD].steps(ctrl._cfg)
    _set(
        mp,
        S.ENTER_UNLOAD,
        steps=_steps(*(s for s in steps if getattr(s, "label", "") != "gripper open")),
    )


def _close_gripper(cell: SimCell) -> None:
    cell.gripper.close()


def m_no_door_wait(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    _set(mp, S.OPEN_DOOR_LOAD, steps=_steps(Command("open door", lambda c: c.cnc.open_door())))
    _set(mp, S.LOAD, guards=())


def m_trust_one_door_sensor(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    mp.setattr(ctrl._plausibility, "check", lambda: [])
    _set(
        mp,
        S.OPEN_DOOR_LOAD,
        steps=_steps(
            Command("open door", lambda c: c.cnc.open_door()),
            WaitUntil("door open", lambda c: c.cnc.door_open()),
        ),
    )
    _set(mp, S.LOAD, guards=(_door_open_only,))


def m_plausibility_off(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    mp.setattr(ctrl._plausibility, "check", lambda: [])


def m_close_before_retreat(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    _set(mp, S.RETREAT, steps=_steps())
    _set(mp, S.CLOSE_DOOR, guards=())


def m_start_with_arm_inside(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    m_close_before_retreat(mp, ctrl)
    _set(mp, S.MACHINING, guards=())


def m_no_unclamp_wait(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    _set(
        mp,
        S.UNCLAMP,
        steps=_steps(Command("unclamp", lambda c: c.cnc.unclamp()), Move("above_fixture")),
    )


def m_skip_fallback(mp: pytest.MonkeyPatch, ctrl: CellController) -> None:
    _set(
        mp,
        S.UNCLAMP_FALLBACK,
        steps=_steps(Command("unclamp", lambda c: c.cnc.unclamp()), Move("above_fixture")),
    )


@dataclass(frozen=True)
class Broken:
    mutate: Mutation
    faults: tuple[FaultEvent, ...]
    expect: str
    no_release_sensor: bool = False
    # inject when the controller enters the state: a CncFault, or "close_gripper"
    at_state: tuple[State, CncFault | str] | None = None


BROKEN = {
    "keep-entering-while-door-closes": Broken(
        m_no_inside_guard, (), "door not open", at_state=(S.LOAD, CncFault.DOOR_CLOSES_UNCOMMANDED)
    ),
    "enter-with-door-half-open": Broken(
        m_no_door_wait, (FaultEvent(0, CncFault.DOOR_STUCK),), "door not open"
    ),
    "enter-trusting-shorted-door-sensor": Broken(
        m_trust_one_door_sensor, (FaultEvent(0, CncFault.DOOR_SENSOR_SHORT),), "door not open"
    ),
    "door-closes-on-arm": Broken(m_close_before_retreat, (), "door closed on arm"),
    "cycle-start-with-arm-inside": Broken(
        m_start_with_arm_inside, (), "cycle start with arm in machine"
    ),
    "pull-stuck-clamp-without-check": Broken(
        m_no_unclamp_wait,
        (),
        "while still clamped",
        at_state=(S.MACHINING, CncFault.CLAMP_STUCK_ON),
    ),
    "cut-a-crooked-part": Broken(
        m_no_seat_checks,
        (),
        "cycle start with part not seated",
        at_state=(S.PICK_RAW, CncFault.PART_MISSEATED),
    ),
    "enter-fixture-with-gripper-closed": Broken(
        m_no_gripper_open_check,
        (),
        "gripper closed while entering occupied fixture",
        at_state=(S.OPEN_DOOR_UNLOAD, "close_gripper"),
    ),
    "skip-fallback-on-no-sensor-machine": Broken(
        m_skip_fallback, (), "while still clamped", no_release_sensor=True
    ),
}


def _run_cycle(
    cell: SimCell,
    ctrl: CellController,
    faults: tuple[FaultEvent, ...],
    at_state: tuple[State, CncFault | str] | None = None,
) -> None:
    watchman = Watchman(
        cell.cfg, cell.clock, cell.sensors, cell.cnc, ctrl.request_safe, _NoAlerts()
    )
    runner = ControllerRunner(ctrl, watchman)
    pending = [at_state] if at_state else []

    def step() -> bool:
        if pending and ctrl.state is pending[0][0]:
            action = pending.pop()[1]
            if action == "close_gripper":
                _close_gripper(cell)
            elif isinstance(action, CncFault):
                cell.cnc.inject(action)
        return runner.step()

    run_with_faults(cell, step, faults)


@pytest.mark.parametrize("case", BROKEN.values(), ids=BROKEN.keys())
def test_watchdog_catches_broken_controller(case: Broken, monkeypatch: pytest.MonkeyPatch) -> None:
    cell = new_cell(case.no_release_sensor)
    ctrl, _ = make_controller(cell)
    case.mutate(monkeypatch, ctrl)
    _run_cycle(cell, ctrl, case.faults, case.at_state)
    assert any(case.expect in v for v in cell.violations), (cell.violations, ctrl.history)


@pytest.mark.parametrize("case", BROKEN.values(), ids=BROKEN.keys())
def test_same_faults_with_real_controller_are_safe(case: Broken) -> None:
    cell = new_cell(case.no_release_sensor)
    ctrl, _ = make_controller(cell)
    _run_cycle(cell, ctrl, case.faults, case.at_state)
    assert cell.violations == []


def test_door_checks_back_up_plausibility(monkeypatch: pytest.MonkeyPatch) -> None:
    """Defense in depth: with plausibility off, a shorted door sensor is still caught by
    the door_open-and-not-door_closed checks, so the arm never enters."""
    cell = new_cell()
    ctrl, _ = make_controller(cell)
    m_plausibility_off(monkeypatch, ctrl)
    _run_cycle(cell, ctrl, (FaultEvent(0, CncFault.DOOR_SENSOR_SHORT),))
    assert cell.violations == []
    assert ctrl.state is S.SAFE and "OPEN_DOOR_LOAD timeout" in ctrl.last_safe_reason


def test_watchdog_catches_unclamp_during_cycle() -> None:
    cell = new_cell()
    cell.cnc.clamp()
    cell.clock.advance(1)
    cell.cnc.cycle_start()
    assert cell.cnc.cycle_running()
    cell.cnc.unclamp()
    assert any("unclamp during cycle" in v for v in cell.violations)


# --- randomized: the real controller under random fault timing and combinations ---


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
    system = make_system(cell)
    runner = ControllerRunner(system.ctrl, system.watchman)
    run_with_faults(cell, runner.step, schedule)
    assert cell.violations == [], (schedule, cell.violations)
    assert runner.done or runner.stopped, (schedule, system.ctrl.state)
    if runner.stopped:
        assert cell.cnc.feed_hold_active()
        assert cell.robot.status() in (
            RobotStatus.STOPPED,
            RobotStatus.FAULT,
            RobotStatus.FORCE_LIMIT,
        )
        assert len(system.safe_alerts) == 1


def test_inside_guard_catches_door_closing_when_plausibility_is_off(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Defense in depth: the LOAD guard alone still stops the arm going deeper."""
    cell = new_cell()
    ctrl, _ = make_controller(cell)
    m_plausibility_off(monkeypatch, ctrl)
    _run_cycle(cell, ctrl, (), (S.LOAD, CncFault.DOOR_CLOSES_UNCOMMANDED))
    assert cell.violations == []
    assert "guard arm_may_be_inside" in ctrl.last_safe_reason
