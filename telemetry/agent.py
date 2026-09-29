"""Telemetry agent: record events locally, sync to the cloud whenever the network allows.

At-least-once delivery with idempotent ids: an event is removed from the local queue
only after the cloud confirms its id. If the confirmation is lost, the event is sent
again and the cloud's dedupe by id keeps it counted once. Failures back off
exponentially (with upward jitter) up to backoff_max_s, and reset on success.
"""

from __future__ import annotations

import random
from dataclasses import asdict
from typing import Any

from cell.clock import Clock
from cell.controller.alerts import Alert
from telemetry.queue import EventQueue, Priority
from telemetry.uploader import Uploader, UploadError


class TelemetryAgent:
    def __init__(
        self,
        queue: EventQueue,
        uploader: Uploader,
        clock: Clock,
        batch_size: int,
        backoff_initial_s: float,
        backoff_max_s: float,
        seed: int | None = None,
    ) -> None:
        if batch_size <= 0 or backoff_initial_s <= 0 or backoff_max_s < backoff_initial_s:
            raise ValueError("bad telemetry agent settings")
        self.queue = queue
        self._uploader = uploader
        self._clock = clock
        self._batch = batch_size
        self._b0, self._bmax = backoff_initial_s, backoff_max_s
        self._rng = random.Random(seed)
        self.backoff_s = 0.0
        self._next_attempt = 0.0
        self.last_error = ""

    def record(
        self,
        kind: str,
        payload: dict[str, Any],
        priority: Priority = Priority.NORMAL,
        event_id: str | None = None,
    ) -> str:
        return self.queue.put(kind, payload, self._clock.now(), priority, event_id)

    def sync_once(self) -> int:
        """One upload attempt if due. Returns the number of events the cloud confirmed."""
        now = self._clock.now()
        if now < self._next_attempt:
            return 0
        batch = self.queue.pending(self._batch)
        if not batch:
            return 0
        try:
            result = self._uploader.upload(batch)
        except UploadError as e:
            self.last_error = str(e)
            self.backoff_s = min(
                self._bmax, self._b0 if self.backoff_s == 0 else self.backoff_s * 2
            )
            self._next_attempt = now + self.backoff_s * self._rng.uniform(1.0, 1.25)
            return 0
        sent = {e.id for e in batch}
        confirmed = [i for i in result.accepted_ids if i in sent]
        self.queue.mark_sent(confirmed)
        self.backoff_s = 0.0
        self._next_attempt = now
        self.last_error = ""
        return len(confirmed)


class TelemetryAlerter:
    """Alerter that queues every alert as a CRITICAL telemetry event."""

    def __init__(self, agent: TelemetryAgent) -> None:
        self._agent = agent

    def alert(self, alert: Alert) -> None:
        self._agent.record("alert", asdict(alert), Priority.CRITICAL)
