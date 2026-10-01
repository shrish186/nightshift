"""Both implementations must reproduce the locked, reviewed decision timeline."""

from __future__ import annotations

import ctypes
import json

import pytest

from cell.watchman.hardstop import HardStop, HardStopConfig
from tests.hardstop.cbind import CHardStop
from tests.hardstop.golden import PATH, streams, summarize


@pytest.mark.parametrize("impl", ["python", "c"])
def test_matches_locked_golden(impl: str, dll: ctypes.CDLL, hcfg: HardStopConfig) -> None:
    golden = json.loads(PATH.read_text())
    assert golden["config"] == hcfg.__dict__, "config changed: regenerate golden on purpose"
    for s in streams(hcfg):
        hs = HardStop(hcfg) if impl == "python" else CHardStop(dll, hcfg)
        masks = [hs.step(t, a, v, c, h) for t, a, v, c, h in
                 zip(s.t_ms, s.current, s.vib, s.clipped, s.hint, strict=True)]  # fmt: skip
        assert summarize(s.name, masks) == golden["streams"][s.name], s.name
