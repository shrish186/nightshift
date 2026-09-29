"""Fake cloud endpoint for telemetry tests. Dedupes by event id like the real cloud must."""

from __future__ import annotations

import random

from telemetry.queue import Event
from telemetry.uploader import UploadError, UploadResult


class FakeCloud:
    def __init__(self) -> None:
        self.received: dict[str, Event] = {}  # event id -> event (deduped)
        self.deliveries = 0  # every event received, including duplicates
        self.order: list[str] = []

    def accept(self, batch: list[Event]) -> list[str]:
        for e in batch:
            self.deliveries += 1
            if e.id not in self.received:
                self.received[e.id] = e
                self.order.append(e.id)
        return [e.id for e in batch]


class FakeUploader:
    """Network between hub and cloud, with failure modes for tests."""

    def __init__(self, cloud: FakeCloud, seed: int = 0) -> None:
        self.cloud = cloud
        self.online = True
        self.fail_rate = 0.0  # flapping network
        self.lose_ack = False  # server stores the batch, but the reply never arrives
        self.partial: int | None = None  # server accepts only the first N of a batch
        self.calls = 0
        self._rng = random.Random(seed)

    def upload(self, batch: list[Event]) -> UploadResult:
        self.calls += 1
        if not self.online or self._rng.random() < self.fail_rate:
            raise UploadError("network unreachable")
        sent = batch if self.partial is None else batch[: self.partial]
        accepted = self.cloud.accept(sent)
        if self.lose_ack:
            raise UploadError("connection reset after send")
        return UploadResult(accepted)
