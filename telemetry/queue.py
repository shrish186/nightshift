"""Durable, offline-first event queue (SQLite) for the shop hub.

- Every event has an id. put() is idempotent by id (recording the same event twice
  keeps one copy), and the cloud dedupes by the same id, so at-least-once delivery
  counts exactly once.
- Survives restarts and crashes (WAL). ASSUMPTION: synchronous=NORMAL, so a power cut
  can lose the last few ms of events; the hub is on a small UPS (see hardware/bom.md).
- Disk cap: only BULK events (e.g. 10 Hz features) are ever evicted, oldest first.
  CRITICAL (alerts, pauses, audit) and NORMAL events are never dropped.
- Sent events are deleted once the cloud confirms them.
"""

from __future__ import annotations

import json
import sqlite3
import uuid
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Any


class Priority(IntEnum):
    CRITICAL = 0  # alerts, remote pause + acks, audit records: never evicted, sent first
    NORMAL = 1  # state changes, cycle summaries: never evicted
    BULK = 2  # high-rate features: evicted oldest-first under the disk cap


@dataclass(frozen=True)
class Event:
    id: str
    ts: float
    kind: str
    priority: Priority
    payload: dict[str, Any]


_SCHEMA = """
CREATE TABLE IF NOT EXISTS events (
    seq      INTEGER PRIMARY KEY AUTOINCREMENT,
    id       TEXT UNIQUE NOT NULL,
    ts       REAL NOT NULL,
    kind     TEXT NOT NULL,
    priority INTEGER NOT NULL,
    payload  TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS events_send_order ON events (priority, seq);
CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value INTEGER NOT NULL);
"""


class EventQueue:
    def __init__(self, path: str | Path, max_bulk: int) -> None:
        if max_bulk <= 0:
            raise ValueError("max_bulk must be > 0")
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(str(path))
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=NORMAL")
        self._db.executescript(_SCHEMA)
        self._max_bulk = max_bulk

    def put(
        self,
        kind: str,
        payload: dict[str, Any],
        ts: float,
        priority: Priority = Priority.NORMAL,
        event_id: str | None = None,
    ) -> str:
        eid = event_id or uuid.uuid4().hex
        with self._db:
            self._db.execute(
                "INSERT OR IGNORE INTO events (id, ts, kind, priority, payload) "
                "VALUES (?, ?, ?, ?, ?)",
                (eid, ts, kind, int(priority), json.dumps(payload, sort_keys=True)),
            )
            if priority is Priority.BULK:
                self._evict_bulk()
        return eid

    def _evict_bulk(self) -> None:
        (n,) = self._db.execute(
            "SELECT COUNT(*) FROM events WHERE priority = ?", (int(Priority.BULK),)
        ).fetchone()
        excess = n - self._max_bulk
        if excess > 0:
            self._db.execute(
                "DELETE FROM events WHERE seq IN (SELECT seq FROM events WHERE priority = ? "
                "ORDER BY seq LIMIT ?)",
                (int(Priority.BULK), excess),
            )
            self._db.execute(
                "INSERT INTO meta (key, value) VALUES ('evicted', ?) "
                "ON CONFLICT(key) DO UPDATE SET value = value + excluded.value",
                (excess,),
            )

    def pending(self, limit: int) -> list[Event]:
        rows = self._db.execute(
            "SELECT id, ts, kind, priority, payload FROM events ORDER BY priority, seq LIMIT ?",
            (limit,),
        ).fetchall()
        return [Event(r[0], r[1], r[2], Priority(r[3]), json.loads(r[4])) for r in rows]

    def mark_sent(self, ids: list[str]) -> None:
        """Only call with ids the cloud has confirmed."""
        with self._db:
            self._db.executemany("DELETE FROM events WHERE id = ?", [(i,) for i in ids])

    def stats(self) -> dict[str, int]:
        (pending,) = self._db.execute("SELECT COUNT(*) FROM events").fetchone()
        row = self._db.execute("SELECT value FROM meta WHERE key = 'evicted'").fetchone()
        return {"pending": pending, "evicted": row[0] if row else 0}

    def close(self) -> None:
        self._db.close()
