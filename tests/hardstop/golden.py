"""Locked expected output for the config-only streams (edge + fuzz).

Regenerate ONLY when a rule change is intended and reviewed:
    .venv/bin/python -m tests.hardstop.golden
The diff of golden_expected.json is then part of the review.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from cell.config import load_cell_config
from cell.watchman.hardstop import HardStop, HardStopConfig
from tests.conftest import SIM_CELL
from tests.hardstop.streams import Stream, edge_streams, fuzz_streams

PATH = Path(__file__).with_name("golden_expected.json")
BITS = {"CUT_START": 1, "CUT_END": 2, "OVERLOAD": 4, "TOOL_BREAK": 8, "SENSOR_FAULT": 16}


def summarize(name: str, masks: list[int]) -> dict[str, object]:
    out: dict[str, object] = {
        "steps": len(masks),
        "counts": {k: sum(1 for m in masks if m & b) for k, b in BITS.items()},
        "sha256": hashlib.sha256(json.dumps(masks).encode()).hexdigest(),
    }
    if name.startswith("edge-"):
        out["events"] = [[i, m] for i, m in enumerate(masks) if m]
    return out


def streams(cfg: HardStopConfig) -> list[Stream]:
    return edge_streams(cfg) + fuzz_streams()


def main() -> None:
    cfg = HardStopConfig.from_watchman(load_cell_config(SIM_CELL).watchman)
    result = {}
    for s in streams(cfg):
        hs = HardStop(cfg)
        masks = [hs.step(t, a, v, c, h) for t, a, v, c, h in
                 zip(s.t_ms, s.current, s.vib, s.clipped, s.hint, strict=True)]  # fmt: skip
        result[s.name] = summarize(s.name, masks)
    PATH.write_text(json.dumps({"config": cfg.__dict__, "streams": result}, indent=1) + "\n")
    print(f"wrote {PATH} ({len(result)} streams)")


if __name__ == "__main__":
    main()
