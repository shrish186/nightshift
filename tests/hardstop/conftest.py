from __future__ import annotations

import ctypes
from pathlib import Path

import pytest

from cell.config import load_cell_config
from cell.watchman.hardstop import HardStopConfig
from tests.conftest import SIM_CELL
from tests.hardstop.cbind import compile_lib


@pytest.fixture(scope="session")
def dll(tmp_path_factory: pytest.TempPathFactory) -> ctypes.CDLL:
    return compile_lib(Path(tmp_path_factory.mktemp("hardstop")))


@pytest.fixture(scope="session")
def hcfg() -> HardStopConfig:
    return HardStopConfig.from_watchman(load_cell_config(SIM_CELL).watchman)
