"""Watchman sensor interface: spindle current, vibration and sound."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class SensorFrame:
    ts: float  # time the sample was taken, on the cell clock
    spindle_current_a: float
    vibration_rms_g: float
    audio_rms: float


class Sensors(Protocol):
    def read(self) -> SensorFrame | None:
        """Latest frame, or None if the sensor has never produced one.

        Callers must check ts for staleness; a frozen sensor keeps returning its last frame.
        """
