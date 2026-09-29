"""Controller state machine: normal cycle, logging, table structure."""

from __future__ import annotations

import json
import logging
from collections.abc import Callable

import pytest

from cell.controller.alerts import MemoryAlerter
from cell.controller.machine import STATE_TABLE, CellController
from cell.controller.states import State
from cell.drivers.robot import RobotStatus
from cell.drivers.sim.cell import SimCell
from cell.drivers.sim.cnc import CncFault
from cell.log import JsonFormatter
from tests.faults.harness import PROGRAM, new_cell

S = State

NORMAL_CYCLE_WITH_SENSOR = [
    S.IDLE,
    S.PICK_RAW,
    S.OPEN_DOOR_LOAD,
    S.LOAD,
    S.CLAMP,
    S.RETREAT,
    S.CLOSE_DOOR,
    S.MACHINING,
    S.OPEN_DOOR_UNLOAD,
    S.ENTER_UNLOAD,
    S.UNCLAMP,
    S.EXIT,
    S.PLACE_DONE,
    S.IDLE,
]
NORMAL_CYCLE_NO_SENSOR = [
    S.UNCLAMP_FALLBACK if s is S.UNCLAMP else s for s in NORMAL_CYCLE_WITH_SENSOR
]


def make(cell: SimCell) -> tuple[CellController, MemoryAlerter]:
    alerter = MemoryAlerter()
    ctrl = CellController(
        cell.cfg, cell.clock, cell.robot, cell.gripper, cell.cnc, cell.safety, alerter
    )
    return ctrl, alerter


def run_one_cycle(cell: SimCell, ctrl: CellController, max_s: float = 200.0) -> None:
    assert ctrl.start_cycle(PROGRAM)
    start = ctrl.cycles_completed
    t0 = cell.clock.now()
    while cell.clock.now() - t0 < max_s:
        ctrl.step()
        if ctrl.state in (S.IDLE, S.SAFE) and (
            ctrl.cycles_completed > start or ctrl.state is S.SAFE
        ):
            return
        cell.clock.advance(0.1)


def visited(ctrl: CellController) -> list[State]:
    return [ctrl.history[0].src, *(t.dst for t in ctrl.history)]


@pytest.mark.parametrize(
    ("no_release_sensor", "expected"),
    [(False, NORMAL_CYCLE_WITH_SENSOR), (True, NORMAL_CYCLE_NO_SENSOR)],
)
def test_normal_cycle_visits_expected_states(
    no_release_sensor: bool, expected: list[State]
) -> None:
    cell = new_cell(no_release_sensor)
    ctrl, alerter = make(cell)
    run_one_cycle(cell, ctrl)
    assert ctrl.state is S.IDLE, ctrl.last_safe_reason
    assert visited(ctrl) == expected
    assert ctrl.cycles_completed == 1
    assert alerter.alerts == []
    assert cell.violations == []
    assert cell.robot.at_pose() == "home"


def test_back_to_back_cycles() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    for _ in range(3):
        run_one_cycle(cell, ctrl)
    assert ctrl.cycles_completed == 3 and ctrl.state is S.IDLE
    assert cell.violations == []


def test_every_transition_logged_as_json(caplog: pytest.LogCaptureFixture) -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    with caplog.at_level(logging.INFO, logger="cell.controller"):
        run_one_cycle(cell, ctrl)
    lines = [json.loads(JsonFormatter().format(r)) for r in caplog.records]
    transitions = [ln for ln in lines if ln.get("event") == "transition"]
    assert len(transitions) == len(ctrl.history) == len(NORMAL_CYCLE_WITH_SENSOR) - 1
    for ln in transitions:
        assert ln["cell_id"] == "sim-01"
        assert ln["reason"]
        assert isinstance(ln["ts"], float)


def test_start_cycle_only_from_idle() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    ctrl.step()
    assert ctrl.state is S.PICK_RAW
    assert not ctrl.start_cycle(PROGRAM)


def test_idle_does_nothing_without_start() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    for _ in range(100):
        ctrl.step()
        cell.clock.advance(0.1)
    assert ctrl.state is S.IDLE and ctrl.history == []
    assert cell.robot.at_pose() == "home"


# --- table structure ---


def test_every_working_state_has_a_spec() -> None:
    assert set(STATE_TABLE) == set(State) - {S.IDLE, S.SAFE}


def test_every_spec_has_timeout_and_next_state() -> None:
    cfg = new_cell().cfg
    for state, spec in STATE_TABLE.items():
        assert spec.timeout(cfg, next(iter(cfg.programs.values()))) > 0, state
        assert spec.next_states, state
        assert S.SAFE not in spec.next_states  # SAFE is reachable from anywhere, never "next"


def test_enter_unload_can_only_go_to_a_release_check() -> None:
    assert STATE_TABLE[S.ENTER_UNLOAD].next_states == {S.UNCLAMP, S.UNCLAMP_FALLBACK}


@pytest.mark.parametrize(
    ("no_release_sensor", "expected"), [(False, S.UNCLAMP), (True, S.UNCLAMP_FALLBACK)]
)
def test_release_check_branch_follows_config(no_release_sensor: bool, expected: State) -> None:
    cfg = new_cell(no_release_sensor).cfg
    assert STATE_TABLE[S.ENTER_UNLOAD].next(cfg) is expected


def test_only_unclamp_states_lead_to_exit() -> None:
    into_exit = {s for s, spec in STATE_TABLE.items() if S.EXIT in spec.next_states}
    assert into_exit == {S.UNCLAMP, S.UNCLAMP_FALLBACK}


# --- SAFE, reset, and failure handling ---


def go_safe_at_home(cell: SimCell, ctrl: CellController) -> None:
    ctrl.request_safe("test")
    ctrl.step()
    assert ctrl.state is S.SAFE and cell.robot.at_pose() == "home"


def test_safe_is_latched() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    go_safe_at_home(cell, ctrl)
    n = len(ctrl.history)
    for _ in range(50):
        ctrl.step()
        cell.clock.advance(0.1)
    assert ctrl.state is S.SAFE and len(ctrl.history) == n
    assert not ctrl.start_cycle(PROGRAM)


def test_estop_while_idle_goes_safe() -> None:
    cell = new_cell()
    ctrl, alerter = make(cell)
    cell.safety.press_estop()
    ctrl.step()
    assert ctrl.state is S.SAFE and alerter.alerts[0].reason == "e-stop"


@pytest.mark.parametrize(
    ("block", "expect"),
    [
        (lambda c: c.safety.press_estop(), "e-stop not released"),
        (lambda c: c.safety.open_guard(), "guard not closed"),
        (lambda c: c.cnc.inject(CncFault.ALARM), "CNC alarm not cleared"),
        (lambda c: None, "feed hold not cleared"),  # SAFE set feed hold; nobody cleared it
        (lambda c: c.cnc.inject(CncFault.DOOR_SENSOR_SHORT), "sensor plausibility"),
    ],
)
def test_reset_refused_until_cleared(block: Callable[[SimCell], None], expect: str) -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    go_safe_at_home(cell, ctrl)
    block(cell)
    if expect != "feed hold not cleared":
        cell.cnc.operator_clear()
        if expect == "CNC alarm not cleared":
            cell.cnc.inject(CncFault.ALARM)
    why = ctrl.reset("asha")
    assert why is not None and expect in why
    assert ctrl.state is S.SAFE


def test_reset_refused_with_arm_inside_machine() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.CLAMP)
    ctrl.request_safe("test")
    ctrl.step()
    cell.cnc.operator_clear()
    why = ctrl.reset("asha")
    assert why is not None and "inside machine" in why
    assert cell.robot.status() is RobotStatus.STOPPED  # re-latched


def test_reset_refused_when_arm_position_unknown() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    for _ in range(100):  # step until the arm is mid-move
        ctrl.step()
        if cell.robot.status() is RobotStatus.MOVING:
            break
        cell.clock.advance(0.1)
    cell.clock.advance(0.2)
    cell.safety.press_estop()
    ctrl.step()
    cell.safety.release_estop()
    cell.cnc.operator_clear()
    why = ctrl.reset("asha")
    assert why is not None and "unknown" in why


def test_reset_accepted_then_full_cycle_runs() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    go_safe_at_home(cell, ctrl)
    cell.cnc.operator_clear()
    assert ctrl.reset("asha") is None
    assert ctrl.state is S.IDLE and ctrl.history[-1].reason == "reset by asha"
    run_one_cycle(cell, ctrl)
    assert ctrl.state is S.IDLE and ctrl.cycles_completed == 1
    assert cell.violations == []


def test_reset_only_from_safe() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    assert ctrl.reset("asha") == "not in SAFE (state IDLE)"


def test_missing_state_spec_goes_safe(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delitem(STATE_TABLE, S.LOAD)
    cell = new_cell()
    ctrl, _ = make(cell)
    run_one_cycle(cell, ctrl)
    assert ctrl.state is S.SAFE and "no spec" in ctrl.last_safe_reason
    assert cell.violations == []


def test_driver_exception_goes_safe(monkeypatch: pytest.MonkeyPatch) -> None:
    cell = new_cell()
    ctrl, _ = make(cell)

    def boom(pose: str) -> None:
        raise RuntimeError("robot comms lost")

    monkeypatch.setattr(cell.robot, "move_to", boom)
    run_one_cycle(cell, ctrl)
    assert ctrl.state is S.SAFE
    assert "RuntimeError: robot comms lost" in ctrl.last_safe_reason
    assert cell.cnc.feed_hold_active()


def test_failing_stop_action_still_goes_safe(monkeypatch: pytest.MonkeyPatch) -> None:
    cell = new_cell()
    ctrl, alerter = make(cell)

    def broken_stop() -> None:
        raise RuntimeError("no response")

    monkeypatch.setattr(cell.robot, "stop", broken_stop)
    ctrl.request_safe("test")
    ctrl.step()
    assert ctrl.state is S.SAFE
    assert cell.cnc.feed_hold_active()  # feed hold still attempted
    assert "robot stop failed" in alerter.alerts[0].reason


def test_failing_alerter_still_goes_safe() -> None:
    class BrokenAlerter:
        def alert(self, alert: object) -> None:
            raise RuntimeError("network down")

    cell = new_cell()
    ctrl = CellController(
        cell.cfg, cell.clock, cell.robot, cell.gripper, cell.cnc, cell.safety, BrokenAlerter()
    )
    ctrl.request_safe("test")
    ctrl.step()
    assert ctrl.state is S.SAFE and cell.cnc.feed_hold_active()


def run_one_cycle_until(cell: SimCell, ctrl: CellController, state: State) -> None:
    for _ in range(3000):
        ctrl.step()
        if ctrl.state in (state, S.SAFE):
            return
        cell.clock.advance(0.1)


@pytest.mark.parametrize("where", ["inside", "unknown"])
def test_reset_never_unlatches_robot_unless_arm_is_outside(
    where: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    if where == "inside":
        run_one_cycle_until(cell, ctrl, S.CLAMP)
        ctrl.request_safe("test")
        ctrl.step()
    else:
        for _ in range(100):
            ctrl.step()
            if cell.robot.status() is RobotStatus.MOVING:
                break
            cell.clock.advance(0.1)
        cell.clock.advance(0.2)
        ctrl.request_safe("test")
        ctrl.step()
    cell.cnc.operator_clear()
    calls: list[int] = []
    real_reset = cell.robot.reset

    def spy_reset() -> None:
        calls.append(1)
        real_reset()

    monkeypatch.setattr(cell.robot, "reset", spy_reset)
    assert ctrl.reset("asha") is not None
    assert calls == []  # the robot was never unlatched
    assert cell.robot.status() is RobotStatus.STOPPED


def test_feed_hold_pressed_mid_cut_goes_safe_at_once() -> None:
    cell = new_cell()
    ctrl, alerter = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.MACHINING)
    for _ in range(30):  # 3 s into the cut
        ctrl.step()
        cell.clock.advance(0.1)
    assert cell.cnc.cycle_running() and ctrl.state is S.MACHINING
    cell.cnc.feed_hold()  # operator presses feed hold on the machine panel
    ctrl.step()
    assert ctrl.state.value == S.SAFE.value  # (.value: mypy keeps the narrowing above)
    assert "feed hold during machining" in alerter.alerts[0].reason


# --- per-program machining timeout ---


def test_hung_cycle_times_out_at_program_expected_time_times_factor() -> None:
    cell = new_cell(cycle_s=10.0)
    ctrl, _ = make(cell)
    cell.cnc.inject(CncFault.CYCLE_HANG)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.MACHINING)
    entered = cell.clock.now()
    while ctrl.state is S.MACHINING and cell.clock.now() - entered < 100:
        ctrl.step()
        cell.clock.advance(0.1)
    limit = 10.0 * cell.cfg.timeouts_s.machining_factor
    assert "MACHINING timeout" in ctrl.last_safe_reason
    assert limit <= cell.clock.now() - entered <= limit + 0.3


def test_unknown_program_is_refused() -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    assert not ctrl.start_cycle("NOPE")
    ctrl.step()
    assert ctrl.state is S.IDLE and ctrl.history == []


# --- command-aware plausibility, mid-stroke (rule a) ---


@pytest.mark.parametrize(
    ("state", "fault", "expect"),
    [
        (S.OPEN_DOOR_LOAD, CncFault.DOOR_SENSORS_STUCK_OPEN, "door_confirmed_too_fast"),
        (S.CLAMP, CncFault.CLAMP_SENSORS_STUCK_CLAMPED, "clamp_confirmed_too_fast"),
    ],
)
def test_stuck_on_dead_pair_mid_stroke_is_caught_before_the_arm_acts(
    state: State, fault: CncFault, expect: str
) -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, state)
    for _ in range(3):  # the command goes out and the stroke starts
        ctrl.step()
        cell.clock.advance(0.05)
    cell.cnc.inject(fault)
    ctrl.step()
    assert ctrl.state.value == S.SAFE.value, ctrl.last_safe_reason
    assert expect in ctrl.last_safe_reason
    assert cell.violations == []


def test_fault_revealed_by_a_command_stops_before_the_next_step() -> None:
    """No released sensor + shorted clamped sensor: 'clamped' already reads True when the
    clamp is commanded. The gripper must not let go of the part in that same tick."""
    cell = new_cell(no_release_sensor=True)
    ctrl, _ = make(cell)
    cell.cnc.inject(CncFault.CLAMP_SENSOR_SHORT)
    run_one_cycle(cell, ctrl)
    assert ctrl.state is S.SAFE
    assert "clamp_confirmed_without_travel" in ctrl.last_safe_reason
    assert cell.violations == []


def test_arm_waits_worst_case_stroke_time_even_if_sensor_sticks_late() -> None:
    """Open stuck on + closed dead appearing 1.4 s into a 2 s stroke looks like a quick
    real stroke. The arm must still wait for the worst-case stroke time before entering."""
    cell = new_cell()
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.OPEN_DOOR_LOAD)
    ctrl.step()  # open command goes out
    for _ in range(14):
        cell.clock.advance(0.1)
        ctrl.step()
    assert cell.cnc.door() == "moving"
    cell.cnc.inject(CncFault.DOOR_SENSORS_STUCK_OPEN)
    run_one_cycle_until(cell, ctrl, S.CLAMP)
    assert cell.violations == []


@pytest.mark.parametrize("how", ["door left open", "stuck-open pair on a closed door"])
def test_reset_refused_unless_door_reads_closed(how: str) -> None:
    cell = new_cell()
    ctrl, _ = make(cell)
    if how == "door left open":
        assert ctrl.start_cycle(PROGRAM)
        run_one_cycle_until(cell, ctrl, S.PLACE_DONE)  # door open, arm outside
        while cell.robot.at_pose() != "home" and ctrl.state is S.PLACE_DONE:
            ctrl.step()
            cell.clock.advance(0.1)
        ctrl.request_safe("test")
        ctrl.step()
    else:
        go_safe_at_home(cell, ctrl)
        cell.cnc.inject(CncFault.DOOR_SENSORS_STUCK_OPEN)
    cell.cnc.operator_clear()
    why = ctrl.reset("asha")
    assert why is not None and "door not confirmed closed" in why


# --- single-sensor clamp: tug test before letting go ---


def test_tug_test_catches_clamp_failure_hidden_by_late_sensor_short() -> None:
    """Hypothesis found this: no released sensor, the clamp never closes, and the one
    clamped sensor shorts on ~0.6 s after the command, which looks like a clean stroke.
    The tug test must find the part is not held, before the gripper lets go."""
    cell = new_cell(no_release_sensor=True)
    ctrl, _ = make(cell)
    cell.cnc.inject(CncFault.CLAMP_FAIL)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.CLAMP)
    while cell.cnc.seat() != "seated" or not cell.gripper.has_part():
        ctrl.step()
        cell.clock.advance(0.1)
    for _ in range(6):  # clamp commanded, "stroke" under way
        ctrl.step()
        cell.clock.advance(0.1)
    cell.cnc.inject(CncFault.CLAMP_SENSOR_SHORT)
    shorted_at = cell.clock.now()
    for _ in range(300):
        ctrl.step()
        if ctrl.state is S.SAFE:
            break
        cell.clock.advance(0.1)
    assert ctrl.state is S.SAFE
    assert "clamp did not hold the part (tug test)" in ctrl.last_safe_reason
    assert cell.clock.now() - shorted_at < 2.0  # the tug, not the clamp timeout
    assert cell.violations == []
    assert cell.gripper.has_part()  # never let go


def test_normal_single_sensor_cycle_passes_the_tug_test() -> None:
    cell = new_cell(no_release_sensor=True)
    ctrl, _ = make(cell)
    run_one_cycle(cell, ctrl)
    assert ctrl.state is S.IDLE and cell.violations == []


# --- part-present debounce ---


def _cut_for(cell: SimCell, ctrl: CellController, seconds: float) -> None:
    for _ in range(round(seconds / 0.1)):
        ctrl.step()
        cell.clock.advance(0.1)


def test_seat_sensor_flicker_during_cut_does_not_stop() -> None:
    cell = new_cell(cycle_s=20.0)
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.MACHINING)
    cell.cnc.inject(CncFault.PART_SENSOR_FLICKER)
    while ctrl.state is S.MACHINING:
        _cut_for(cell, ctrl, 0.1)
    assert ctrl.state is S.OPEN_DOOR_UNLOAD, ctrl.last_safe_reason


def test_part_knocked_crooked_mid_cut_stops_just_after_the_window() -> None:
    cell = new_cell(cycle_s=20.0)
    ctrl, _ = make(cell)
    window = cell.cfg.io_plausibility.part_present_max_ignore_s
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.MACHINING)
    _cut_for(cell, ctrl, 3)
    cell.cnc.inject(CncFault.PART_MISSEATED)
    t0 = cell.clock.now()
    while ctrl.state is S.MACHINING:
        _cut_for(cell, ctrl, 0.1)
    assert "part not seated" in ctrl.last_safe_reason
    assert window <= cell.clock.now() - t0 <= window + 0.25


def test_flicker_cannot_fake_an_empty_fixture() -> None:
    """A seated part must never read 'empty' just because the sensor blinked."""
    cell = new_cell()
    ctrl, _ = make(cell)
    cell.cnc.place_part()  # a part left in the fixture
    cell.cnc.inject(CncFault.PART_SENSOR_FLICKER)
    run_one_cycle(cell, ctrl)
    assert ctrl.state is S.SAFE
    assert "fixture empty" in ctrl.last_safe_reason or "LOAD timeout" in ctrl.last_safe_reason
    assert cell.violations == []


def test_dropout_streak_is_not_inherited_from_an_earlier_state() -> None:
    """The seat filter last saw 'no part' back in LOAD; a flicker at the very start of
    the cut must not count all that time as one long dropout."""
    cell = new_cell(cycle_s=20.0)
    ctrl, _ = make(cell)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.MACHINING)
    while not cell.cnc.cycle_running():
        _cut_for(cell, ctrl, 0.1)
    cell.cnc.inject(CncFault.PART_SENSOR_FLICKER)  # drops out right as the cut begins
    _cut_for(cell, ctrl, 1.0)
    assert ctrl.state is S.MACHINING, ctrl.last_safe_reason


# --- hardware door-zone interlock (safety relay + safety-rated door switch) ---


def test_interlock_blocks_entry_when_door_jammed_and_sensor_stuck_open() -> None:
    """Hypothesis found it: door jammed half-way + open sensor stuck on 1.1 s into the
    stroke fools every software check. The hardware interlock must stop the arm."""
    cell = new_cell()
    ctrl, _ = make(cell)
    cell.cnc.inject(CncFault.DOOR_STUCK)
    assert ctrl.start_cycle(PROGRAM)
    run_one_cycle_until(cell, ctrl, S.OPEN_DOOR_LOAD)
    ctrl.step()  # open command
    _cut_for(cell, ctrl, 1.1)
    cell.cnc.inject(CncFault.DOOR_SENSORS_STUCK_OPEN)
    run_one_cycle_until(cell, ctrl, S.CLAMP)
    assert ctrl.state is S.SAFE
    assert cell.violations == []
    assert cell.interlock_blocks >= 1


def test_interlock_fault_reported_by_the_safety_relay_goes_safe() -> None:
    from cell.drivers.sim.safety import SafetyFault

    cell = new_cell()
    ctrl, _ = make(cell)
    cell.safety.inject(SafetyFault.ZONE_INTERLOCK_FAULT)
    ctrl.step()
    assert ctrl.state is S.SAFE and "door zone interlock" in ctrl.last_safe_reason
