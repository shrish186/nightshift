"""Structured JSON logging. Every state transition goes through log_transition()."""

from __future__ import annotations

import json
import logging
from typing import Any


class JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        fields = getattr(record, "fields", None)
        if isinstance(fields, dict):
            payload.update(fields)
        return json.dumps(payload, sort_keys=True)


def get_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if not any(isinstance(h.formatter, JsonFormatter) for h in logger.handlers):
        handler = logging.StreamHandler()
        handler.setFormatter(JsonFormatter())
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = True
    return logger


def log_transition(
    logger: logging.Logger, src: str, dst: str, reason: str, ts: float, cell_id: str
) -> None:
    logger.info(
        "transition",
        extra={
            "fields": {
                "event": "transition",
                "from": src,
                "to": dst,
                "reason": reason,
                "ts": ts,
                "cell_id": cell_id,
            }
        },
    )
