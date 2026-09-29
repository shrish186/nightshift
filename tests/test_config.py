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
    "tug_in_fixture": [650, 0, 184],
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


def _no_release_sensor(raw: dict[str, Any]) -> dict[str, Any]:
    raw = copy.deepcopy(raw)
    raw["machine"]["clamp_released_sensor"] = False
    raw["pins"]["unclamped_in"] = None
    raw["unclamp_fallback"] = {
        "release_wait_s": 2.0,
        "pull_pose": "above_fixture",
        "pull_force_limit_n": 40.0,
        "tug_pose": "tug_in_fixture",
        "tug_force_n": 20.0,
    }
    return raw


def test_clamp_released_sensor_must_be_declared(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    del raw["machine"]["clamp_released_sensor"]
    with pytest.raises(ValidationError, match="clamp_released_sensor"):
        CellConfig.model_validate(raw)


def test_clamp_released_sensor_must_be_a_real_bool(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["machine"]["clamp_released_sensor"] = "yes"
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)


def test_machine_without_release_sensor_loads_with_fallback(sim_cell_raw: dict[str, Any]) -> None:
    cfg = CellConfig.model_validate(_no_release_sensor(sim_cell_raw))
    assert cfg.unclamp_fallback is not None
    assert cfg.unclamp_fallback.pull_force_limit_n == 40.0


def test_no_release_sensor_requires_fallback(sim_cell_raw: dict[str, Any]) -> None:
    raw = _no_release_sensor(sim_cell_raw)
    del raw["unclamp_fallback"]
    with pytest.raises(ValidationError, match="unclamp_fallback is required"):
        CellConfig.model_validate(raw)


def test_release_sensor_forbids_fallback(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["unclamp_fallback"] = _no_release_sensor(sim_cell_raw)["unclamp_fallback"]
    with pytest.raises(ValidationError, match="must be absent"):
        CellConfig.model_validate(raw)


def test_release_sensor_requires_its_pin(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["pins"]["unclamped_in"] = None
    with pytest.raises(ValidationError, match="unclamped_in is null"):
        CellConfig.model_validate(raw)


def test_unclamped_pin_key_cannot_be_omitted(sim_cell_raw: dict[str, Any]) -> None:
    raw = _no_release_sensor(sim_cell_raw)
    del raw["pins"]["unclamped_in"]
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)


def test_fallback_pull_pose_must_exist(sim_cell_raw: dict[str, Any]) -> None:
    raw = _no_release_sensor(sim_cell_raw)
    raw["unclamp_fallback"]["pull_pose"] = "nowhere"
    with pytest.raises(ValidationError, match="pull_pose"):
        CellConfig.model_validate(raw)


@pytest.mark.parametrize("pair", ["door", "clamp"])
@pytest.mark.parametrize("delta", [0.0, -0.1])
def test_plausibility_window_must_exceed_travel(
    sim_cell_raw: dict[str, Any], pair: str, delta: float
) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    p = raw["io_plausibility"]
    p[f"{pair}_plausibility_window_s"] = p[f"{pair}_travel_s"] + delta
    with pytest.raises(ValidationError, match=f"{pair}_plausibility_window_s must be >"):
        CellConfig.model_validate(raw)


def test_plausibility_section_required(sim_cell_raw: dict[str, Any]) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    del raw["io_plausibility"]
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)


def test_unclamp_timeout_must_exceed_fallback_wait(sim_cell_raw: dict[str, Any]) -> None:
    raw = _no_release_sensor(sim_cell_raw)
    raw["timeouts_s"]["unclamp"] = raw["unclamp_fallback"]["release_wait_s"]
    with pytest.raises(ValidationError, match=r"timeouts_s\.unclamp must be >"):
        CellConfig.model_validate(raw)


@pytest.mark.parametrize("kind", ["chip", "wear"])
def test_watchman_stop_ratio_must_exceed_alert(sim_cell_raw: dict[str, Any], kind: str) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["watchman"][f"{kind}_stop_ratio"] = raw["watchman"][f"{kind}_alert_ratio"]
    with pytest.raises(ValidationError, match=f"{kind}_stop_ratio must be >"):
        CellConfig.model_validate(raw)


@pytest.mark.parametrize("value", [0.0, 1.5])
def test_part_present_ignore_window_bounds(sim_cell_raw: dict[str, Any], value: float) -> None:
    raw = copy.deepcopy(sim_cell_raw)
    raw["io_plausibility"]["part_present_max_ignore_s"] = value
    with pytest.raises(ValidationError):
        CellConfig.model_validate(raw)
