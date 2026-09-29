"""Watchman references: supervised recording, operator confirmation, per program + tool,
tool change, and the unattended start gate."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from cell.config import CellConfig
from cell.controller.states import State
from cell.drivers.sim.cell import SimCell
from cell.watchman.reference import CycleStats, Reference, ReferenceStore
from tests.faults.harness import (
    OPERATOR,
    PROGRAM,
    TOOL,
    ControllerRunner,
    System,
    make_system,
    new_cell,
    run_with_faults,
)


def fresh() -> tuple[SimCell, System]:
    cell = new_cell()
    return cell, make_system(cell, with_reference=False)


def supervised_cycle(cell: SimCell, system: System) -> ControllerRunner:
    runner = ControllerRunner(system.ctrl, system.watchman, supervised=True)
    run_with_faults(cell, runner.step)
    assert runner.done, system.ctrl.last_safe_reason
    return runner


def record(cell: SimCell, system: System) -> None:
    w = system.watchman
    assert w.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=True) is None
    while w.recording_progress is not None:
        supervised_cycle(cell, system)
        assert w.confirm_cycle_clean(OPERATOR) is None


def test_unattended_start_refused_without_a_reference() -> None:
    _, system = fresh()
    assert not system.ctrl.start_cycle(PROGRAM)
    assert "no confirmed watchman reference" in system.ctrl.last_start_refusal
    assert system.ctrl.state is State.IDLE


def test_supervised_start_allowed_without_a_reference() -> None:
    cell, system = fresh()
    supervised_cycle(cell, system)


def test_clean_supervised_cycles_are_never_auto_learned() -> None:
    cell, system = fresh()
    for _ in range(6):
        supervised_cycle(cell, system)
    assert system.watchman.reference_for(PROGRAM) is None
    assert not system.ctrl.start_cycle(PROGRAM)


def test_recording_needs_a_confirmed_fresh_tool() -> None:
    _, system = fresh()
    why = system.watchman.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=False)
    assert why is not None and "fresh tool" in why
    assert system.watchman.recording_progress is None


def test_confirm_refused_without_a_new_completed_cycle() -> None:
    cell, system = fresh()
    w = system.watchman
    assert w.confirm_cycle_clean(OPERATOR) == "not recording a reference"
    w.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=True)
    assert w.confirm_cycle_clean(OPERATOR) == "no new completed cycle to confirm"
    supervised_cycle(cell, system)
    assert w.confirm_cycle_clean(OPERATOR) is None
    assert w.confirm_cycle_clean(OPERATOR) == "no new completed cycle to confirm"  # no double


def test_rejected_cycle_is_never_used() -> None:
    cell, system = fresh()
    w = system.watchman
    w.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=True)
    supervised_cycle(cell, system)
    w.reject_cycle()
    assert w.confirm_cycle_clean(OPERATOR) == "no new completed cycle to confirm"
    assert w.recording_progress == (0, cell.cfg.watchman.reference_cycles)


def test_recording_then_unattended_runs_allowed() -> None:
    cell, system = fresh()
    record(cell, system)
    ref = system.watchman.reference_for(PROGRAM)
    assert ref is not None and ref.operator == OPERATOR and ref.fresh_tool_confirmed
    assert ref.tool == TOOL and len(ref.cycles) == cell.cfg.watchman.reference_cycles
    runner = ControllerRunner(system.ctrl, system.watchman)
    run_with_faults(cell, runner.step)
    assert runner.done


def test_unattended_start_refused_while_recording() -> None:
    _, system = make_system_with_reference()
    system.watchman.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=True)
    assert not system.ctrl.start_cycle(PROGRAM)
    assert "recording in progress" in system.ctrl.last_start_refusal


def make_system_with_reference() -> tuple[SimCell, System]:
    cell = new_cell()
    return cell, make_system(cell)


def test_tool_change_drops_its_references_only() -> None:
    _, system = make_system_with_reference()
    other = Reference("P9", "T9", OPERATOR, True, "x", tuple(CycleStats(12, 1) for _ in range(5)))
    system.watchman._store.put(other)
    dropped = system.watchman.tool_changed(TOOL, OPERATOR)
    assert [(r.program, r.tool) for r in dropped] == [(PROGRAM, TOOL)]
    assert system.watchman._store.get("P9", "T9") is not None
    assert not system.ctrl.start_cycle(PROGRAM)  # must re-record after the tool change


def test_store_persists_and_refuses_unconfirmed_tool(tmp_path: Path) -> None:
    path = tmp_path / "refs.json"
    store = ReferenceStore(path)
    ref = Reference(PROGRAM, TOOL, OPERATOR, True, "2026-09-29T00:00:00+00:00",
                    tuple(CycleStats(12.0 + i * 0.1, 1.0) for i in range(5)))  # fmt: skip
    store.put(ref)
    assert ReferenceStore(path).get(PROGRAM, TOOL) == ref
    assert json.loads(path.read_text())["references"][0]["operator"] == OPERATOR
    with pytest.raises(ValueError, match="fresh tool"):
        store.put(Reference(PROGRAM, "T2", OPERATOR, False, "x", ref.cycles))


def test_reference_cycles_must_be_at_least_three() -> None:
    raw = new_cell().cfg.model_dump()
    raw["watchman"]["reference_cycles"] = 2
    with pytest.raises(ValueError):
        CellConfig.model_validate(raw)
