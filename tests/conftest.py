from __future__ import annotations

from datetime import UTC
from pathlib import Path
from typing import Any

import pytest
import yaml

SIM_CELL = Path(__file__).resolve().parent.parent / "cells" / "sim-01.yaml"


@pytest.fixture
def sim_cell_raw() -> dict[str, Any]:
    with open(SIM_CELL) as f:
        raw: dict[str, Any] = yaml.safe_load(f)
    return raw


def pytest_terminal_summary(terminalreporter: Any) -> None:
    """Print and save the false-stop rate so it can be tracked across runs."""
    import json
    from datetime import datetime

    from tests.faults.harness import FALSE_STOP_REPORT

    if not FALSE_STOP_REPORT:
        return
    terminalreporter.section("false-stop rate (no faults injected)")
    for runner, r in FALSE_STOP_REPORT.items():
        terminalreporter.write_line(
            f"{runner}: {r['stops']} stops / {r['cycles']} cycles "
            f"= {r['stop_rate']:.2%} over {r['runs']} runs  {r['reasons'] or ''}"
        )
    out = Path(__file__).resolve().parent.parent / "data" / "reports"
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    (out / f"false_stops_{stamp}.json").write_text(json.dumps(FALSE_STOP_REPORT, indent=2))
