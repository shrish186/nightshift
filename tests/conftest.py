from __future__ import annotations

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
