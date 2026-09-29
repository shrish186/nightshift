"""Telemetry settings (hub). The token is never in config: it comes from .env."""

from __future__ import annotations

from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field


class TelemetryConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    endpoint: str  # https URL of the cloud ingest API
    token_env: str  # name of the env var holding the token (set in the hub's .env)
    queue_path: str  # SQLite file on the hub's SSD
    batch_size: int = Field(gt=0, le=5000)
    backoff_initial_s: float = Field(gt=0)
    backoff_max_s: float = Field(gt=0)
    max_bulk_events: int = Field(gt=0)  # disk cap for BULK (features); others never evicted


def load_telemetry_config(path: str | Path) -> TelemetryConfig:
    with open(path) as f:
        return TelemetryConfig.model_validate(yaml.safe_load(f)["telemetry"])
