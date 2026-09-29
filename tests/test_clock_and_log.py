from __future__ import annotations

import json
import logging

import pytest

from cell.clock import SimClock
from cell.log import JsonFormatter, log_transition


def test_sim_clock_only_moves_when_told() -> None:
    clock = SimClock()
    assert clock.now() == 0.0
    clock.sleep(1.5)
    clock.advance(0.5)
    assert clock.now() == 2.0
    with pytest.raises(ValueError):
        clock.advance(-1)


def test_transition_is_logged_as_json(caplog: pytest.LogCaptureFixture) -> None:
    logger = logging.getLogger("test.transition")
    with caplog.at_level(logging.INFO, logger="test.transition"):
        log_transition(logger, "IDLE", "PICK_RAW", "cycle requested", 12.5, "sim-01")
    line = json.loads(JsonFormatter().format(caplog.records[0]))
    assert line == {
        "level": "INFO",
        "logger": "test.transition",
        "msg": "transition",
        "event": "transition",
        "from": "IDLE",
        "to": "PICK_RAW",
        "reason": "cycle requested",
        "ts": 12.5,
        "cell_id": "sim-01",
    }
