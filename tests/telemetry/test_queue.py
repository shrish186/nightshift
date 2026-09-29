from __future__ import annotations

from pathlib import Path

import pytest

from telemetry.queue import EventQueue, Priority


@pytest.fixture
def q(tmp_path: Path) -> EventQueue:
    return EventQueue(tmp_path / "q.db", max_bulk=100)


def test_put_is_idempotent_by_event_id(q: EventQueue) -> None:
    a = q.put("alert", {"x": 1}, ts=1.0, event_id="e1")
    b = q.put("alert", {"x": 1}, ts=1.0, event_id="e1")
    assert a == b == "e1"
    assert q.stats()["pending"] == 1


def test_generated_ids_are_unique(q: EventQueue) -> None:
    ids = {q.put("features", {"i": i}, ts=float(i)) for i in range(50)}
    assert len(ids) == 50


def test_pending_is_fifo_and_mark_sent_removes(q: EventQueue) -> None:
    ids = [q.put("features", {"i": i}, ts=float(i)) for i in range(10)]
    batch = q.pending(4)
    assert [e.id for e in batch] == ids[:4]
    q.mark_sent([e.id for e in batch])
    assert [e.id for e in q.pending(100)] == ids[4:]


def test_survives_restart(tmp_path: Path) -> None:
    path = tmp_path / "q.db"
    q1 = EventQueue(path, max_bulk=100)
    ids = [q1.put("alert", {"i": i}, ts=float(i), priority=Priority.CRITICAL) for i in range(5)]
    q1.mark_sent(ids[:2])
    q1.close()
    q2 = EventQueue(path, max_bulk=100)
    assert [e.id for e in q2.pending(10)] == ids[2:]
    assert q2.pending(10)[0].payload == {"i": 2}


def test_bulk_is_evicted_first_and_critical_never(tmp_path: Path) -> None:
    q = EventQueue(tmp_path / "q.db", max_bulk=10)
    crit = [q.put("alert", {"i": i}, ts=float(i), priority=Priority.CRITICAL) for i in range(20)]
    bulk = [q.put("features", {"i": i}, ts=float(i), priority=Priority.BULK) for i in range(30)]
    pending = {e.id for e in q.pending(1000)}
    assert set(crit) <= pending  # every critical event kept
    assert set(bulk[-10:]) <= pending  # newest bulk kept
    assert not (set(bulk[:20]) & pending)  # oldest bulk dropped
    assert q.stats()["evicted"] == 20


def test_critical_events_go_out_before_bulk(q: EventQueue) -> None:
    q.put("features", {}, ts=1.0, priority=Priority.BULK)
    alert = q.put("alert", {}, ts=2.0, priority=Priority.CRITICAL)
    assert q.pending(1)[0].id == alert


def test_example_config_loads() -> None:
    from telemetry.config import load_telemetry_config

    cfg = load_telemetry_config(
        Path(__file__).resolve().parents[2] / "config/telemetry.example.yaml"
    )
    assert cfg.endpoint.startswith("https://") and cfg.token_env == "NIGHTSHIFT_TELEMETRY_TOKEN"
