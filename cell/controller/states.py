"""Cell controller states. SAFETY-RELEVANT: changes here need human review."""

from __future__ import annotations

from enum import Enum


class State(Enum):
    IDLE = "IDLE"
    PICK_RAW = "PICK_RAW"
    OPEN_DOOR_LOAD = "OPEN_DOOR_LOAD"
    LOAD = "LOAD"
    CLAMP = "CLAMP"
    RETREAT = "RETREAT"
    CLOSE_DOOR = "CLOSE_DOOR"
    MACHINING = "MACHINING"
    OPEN_DOOR_UNLOAD = "OPEN_DOOR_UNLOAD"
    ENTER_UNLOAD = "ENTER_UNLOAD"
    UNCLAMP = "UNCLAMP"  # machine has a clamp-released sensor
    UNCLAMP_FALLBACK = "UNCLAMP_FALLBACK"  # no sensor: wait + force-limited pull
    EXIT = "EXIT"
    PLACE_DONE = "PLACE_DONE"
    SAFE = "SAFE"
