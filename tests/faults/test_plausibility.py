from __future__ import annotations

import pytest

from cell.config import CellConfig, load_cell_config
from cell.controller.plausibility import IoPlausibilityMonitor, PlausibilityFault
from cell.drivers.sim.cell import SimCell, build_sim_cell
from cell.drivers.sim.cnc import CncFault
from tests.conftest import SIM_CELL

F = PlausibilityFault


@pytest.fixture
def cfg() -> CellConfig:
    return load_cell_config(SIM_CELL)


@pytest.fixture
def cell(cfg: CellConfig) -> SimCell:
    return build_sim_cell(cfg)


def _monitor(cell: SimCell) -> IoPlausibilityMonitor:
    return IoPlausibilityMonitor(cell.cnc, cell.clock, cell.cfg)


def _run(cell: SimCell, mon: IoPlausibilityMonitor, seconds: float) -> set[F]:
    seen: set[F] = set()
    t = 0.0
    while t < seconds:
        seen.update(mon.check())
        cell.clock.advance(0.05)
        t += 0.05
    seen.update(mon.check())
    return seen


def test_normal_door_and_clamp_strokes_are_plausible(cell: SimCell) -> None:
    mon = _monitor(cell)
    mon.cnc.open_door()
    assert _run(cell, mon, 5) == set()
    mon.cnc.clamp()
    assert _run(cell, mon, 2) == set()
    mon.cnc.unclamp()
    assert _run(cell, mon, 2) == set()
    mon.cnc.close_door()
    assert _run(cell, mon, 5) == set()


def test_door_sensor_short_faults_immediately(cell: SimCell) -> None:
    mon = _monitor(cell)
    cell.cnc.inject(CncFault.DOOR_SENSOR_SHORT)
    assert F.DOOR_BOTH_ON in mon.check()


def test_clamp_sensor_short_faults_immediately(cell: SimCell) -> None:
    mon = _monitor(cell)
    cell.cnc.inject(CncFault.CLAMP_SENSOR_SHORT)
    assert F.CLAMP_BOTH_ON in mon.check()


def test_stuck_door_faults_just_after_window_not_before(cell: SimCell) -> None:
    mon = _monitor(cell)
    window = cell.cfg.io_plausibility.door_plausibility_window_s
    cell.cnc.inject(CncFault.DOOR_STUCK)
    mon.cnc.open_door()
    assert mon.check() == []  # timer starts: both sensors off
    cell.clock.advance(window - 0.01)
    assert mon.check() == []
    cell.clock.advance(0.02)
    assert F.DOOR_BOTH_OFF_TOO_LONG in mon.check()


def test_clamp_that_never_closes_trips_origin_stuck(cell: SimCell) -> None:
    # CLAMP_FAIL leaves the jaws open, so "released" stays on after the clamp command.
    mon = _monitor(cell)
    cell.cnc.inject(CncFault.CLAMP_FAIL)
    mon.check()
    mon.cnc.clamp()
    seen = _run(cell, mon, cell.cfg.io_plausibility.clamp_plausibility_window_s + 1)
    assert F.CLAMP_ORIGIN_STUCK in seen
    assert not cell.cnc.clamped()


def test_clamp_stuck_mid_stroke_faults() -> None:
    cfg = load_cell_config(SIM_CELL)
    cell = build_sim_cell(cfg)
    mon = _monitor(cell)
    mon.cnc.clamp()
    cell.clock.advance(0.1)  # jaws moving: both clamp sensors off
    cell.cnc.inject(CncFault.CLAMP_JAM)
    assert mon.check() == []
    cell.clock.advance(cfg.io_plausibility.clamp_plausibility_window_s + 0.1)
    assert F.CLAMP_BOTH_OFF_TOO_LONG in mon.check()


def test_io_power_loss_faults(cell: SimCell) -> None:
    mon = _monitor(cell)
    cell.cnc.inject(CncFault.IO_POWER_LOSS)
    assert mon.check() == []  # both off could still be travel...
    cell.clock.advance(cell.cfg.io_plausibility.door_plausibility_window_s + 0.1)
    faults = set(mon.check())
    assert {F.DOOR_BOTH_OFF_TOO_LONG, F.CLAMP_BOTH_OFF_TOO_LONG} <= faults


def test_io_power_loss_reads_fail_safe(cell: SimCell) -> None:
    cell.cnc.inject(CncFault.IO_POWER_LOSS)
    assert not cell.cnc.door_open() and not cell.cnc.door_closed()
    assert not cell.cnc.clamped() and not cell.cnc.unclamped()
    assert cell.cnc.alarm() and cell.cnc.feed_hold_active() and cell.cnc.cycle_running()
    assert not cell.cnc.cycle_done()


def test_clamp_pair_ignored_without_release_sensor(cfg: CellConfig) -> None:
    raw = cfg.model_dump()
    raw["machine"]["clamp_released_sensor"] = False
    raw["pins"]["unclamped_in"] = None
    raw["unclamp_fallback"] = {
        "release_wait_s": 2.0,
        "pull_pose": "above_fixture",
        "pull_force_limit_n": 40.0,
    }
    cell = build_sim_cell(CellConfig.model_validate(raw))
    mon = _monitor(cell)
    # Unclamped: clamped=False and unclamped=False (no sensor) forever. Not a fault.
    assert _run(cell, mon, 30) == set()


def test_timer_resets_when_a_sensor_comes_back(cell: SimCell) -> None:
    mon = _monitor(cell)
    window = cell.cfg.io_plausibility.door_plausibility_window_s
    mon.cnc.open_door()
    mon.check()
    cell.clock.advance(3)  # door arrives open (DOOR_S=2)
    assert mon.check() == []
    mon.cnc.close_door()
    mon.check()
    cell.clock.advance(window - 0.5)  # closing stroke (2 s) done well within the window
    assert mon.check() == []


# --- command-aware rules ---


def _open_confirmed(cell: SimCell, mon: IoPlausibilityMonitor) -> None:
    mon.cnc.open_door()
    assert _run(cell, mon, 3) == set()
    assert cell.cnc.door_open()


def test_reopening_an_open_door_is_fine(cell: SimCell) -> None:
    mon = _monitor(cell)
    _open_confirmed(cell, mon)
    mon.cnc.open_door()  # already open: a real door does not move
    assert _run(cell, mon, 3) == set()


def test_rule_a_door_confirms_too_fast(cell: SimCell) -> None:
    mon = _monitor(cell)
    mon.check()
    mon.cnc.open_door()
    cell.clock.advance(0.2)
    cell.cnc.inject(CncFault.DOOR_SENSORS_STUCK_OPEN)  # reads open while still travelling
    assert F.DOOR_CONFIRMED_TOO_FAST in mon.check()


def test_rule_a_clamp_confirms_too_fast(cell: SimCell) -> None:
    mon = _monitor(cell)
    mon.check()
    mon.cnc.clamp()
    cell.clock.advance(0.05)
    cell.cnc.inject(CncFault.CLAMP_SENSORS_STUCK_CLAMPED)
    assert F.CLAMP_CONFIRMED_TOO_FAST in mon.check()


def test_rule_b_origin_sensor_never_clears(cell: SimCell) -> None:
    mon = _monitor(cell)
    mon.check()
    cell.cnc.inject(CncFault.DOOR_SENSORS_STUCK_CLOSED)  # looks closed; no reading change
    assert mon.check() == []
    mon.cnc.open_door()
    window = cell.cfg.io_plausibility.door_plausibility_window_s
    assert _run(cell, mon, window - 0.1) == set()
    cell.clock.advance(0.2)
    assert F.DOOR_ORIGIN_STUCK in mon.check()


def test_rule_c_uncommanded_door_motion(cell: SimCell) -> None:
    mon = _monitor(cell)
    _open_confirmed(cell, mon)
    cell.cnc.inject(CncFault.DOOR_CLOSES_UNCOMMANDED)
    assert F.DOOR_UNCOMMANDED_CHANGE in mon.check()


def test_rule_c_stuck_open_pair_appearing_at_rest(cell: SimCell) -> None:
    mon = _monitor(cell)
    mon.check()  # door closed, readings consistent
    cell.cnc.inject(CncFault.DOOR_SENSORS_STUCK_OPEN)
    assert F.DOOR_UNCOMMANDED_CHANGE in mon.check()


def test_rule_d_open_reading_with_no_confirmed_travel(cell: SimCell) -> None:
    cell.cnc.inject(CncFault.DOOR_SENSORS_STUCK_OPEN)  # before the monitor ever looked
    mon = _monitor(cell)
    mon.check()
    mon.cnc.open_door()  # physically closed door, reads open: no travel will be seen
    assert F.DOOR_CONFIRMED_WITHOUT_TRAVEL in mon.check()


def test_rebaseline_accepts_a_consistent_closed_door(cell: SimCell) -> None:
    mon = _monitor(cell)
    _open_confirmed(cell, mon)
    cell.cnc.inject(CncFault.DOOR_CLOSES_UNCOMMANDED)
    cell.clock.advance(3)
    assert F.DOOR_UNCOMMANDED_CHANGE in mon.check()
    mon.rebaseline()  # human reset after checking the cell
    assert mon.check() == []
    _open_confirmed(cell, mon)  # a normal stroke from closed works again
