from __future__ import annotations

import copy
from typing import Any

import pytest
from pydantic import ValidationError

from cell.config import CellConfig, load_cell_config
from tests.conftest import SIM_CELL


def test_sim_cell_loads() -> None:
    cfg = load_cell_config(SIM_CELL)
    assert cfg.cell_id == "sim-01"
    assert cfg.timeouts_s.clamp > 0


def test_missing_pose_refuses_to_start(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    del raw["poses"]["load"]
    with pytest.raises(ValidationError, match="missing poses"):
        CellConfig.model_validate(raw)


@pytest.mark.parametrize("section", ["timeouts_s", "pins", "watchman"])
def test_missing_safety_value_refuses_to_start(sim_cell_raw: dict[str, Any], section: str) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw[section].pop(next(iter(raw[section])))
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)


def test_unknown_key_rejected(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["timeouts_s"]["clmap"] = 5  # typo must not be silently ignored
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)


def test_zero_timeout_rejected(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["timeouts_s"]["clamp"] = 0
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)


def test_zone_pose_must_exist(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["machine_zone_poses"] = ["load", "inside_chuck"]
    with pytest.raises(ValidationError, match="machine_zone_poses"):
        CellConfig.model_validate(raw)


GANTRY_POSES = {
    "home": [0, 0, 400],
    "above_raw_tray": [300, -200, 250],
    "pick_raw": [300, -200, 120],
    "tray_slot_3": [340, -200, 120],
    "above_fixture": [650, 0, 300],
    "load": [650, 0, 180],
    "clear_of_machine": [350, 0, 350],
    "above_done_tray": [300, 200, 250],
    "place_done": [300, 200, 120],
    "safe_home": [0, 0, 450],
}


def test_gantry_config_with_extra_named_poses_loads(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["robot"] = {"kind": "gantry"}
    raw["poses"] = GANTRY_POSES
    cfg = CellConfig.model_validate(raw)
    assert cfg.poses["tray_slot_3"] == (340, -200, 120)


def test_six_axis_pose_rejected_for_gantry(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["robot"] = {"kind": "gantry"}  # poses in sim-01 are 6-axis
    with pytest.raises(ValidationError, match="wrong number of axes"):
        CellConfig.model_validate(raw)


def test_three_axis_pose_rejected_for_cobot(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["poses"]["load"] = [650, 0, 180]
    with pytest.raises(ValidationError, match="wrong number of axes"):
        CellConfig.model_validate(raw)


def test_robot_kind_required(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    del raw["robot"]
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)
