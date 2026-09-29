"""Alerts raised when the cell goes to SAFE. Telemetry/WhatsApp sinks come later."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class Alert:
    ts: float
    cell_id: str
    state: str  # state the cell was in when it went SAFE
    reason: str


class Alerter(Protocol):
    def alert(self, alert: Alert) -> None:
        """Must not raise; implementations queue and retry on their own."""


class MemoryAlerter:
    def __init__(self) -> None:
        self.alerts: list[Alert] = []

    def alert(self, alert: Alert) -> None:
        self.alerts.append(alert)
