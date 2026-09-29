"""Per-cell configuration loaded from cells/<cell-id>.yaml.

Every pose, timeout and pin comes from here. There are deliberately no defaults for
safety-relevant values: a missing key means the cell refuses to start.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, StrictBool, field_validator, model_validator

REQUIRED_POSES = frozenset(
    {
        "home",
        "above_raw_tray",
        "pick_raw",
        "above_fixture",
        "load",
        "clear_of_machine",
        "above_done_tray",
        "place_done",
    }
)

# Pose coordinates are driver-specific and live only in config and the driver.
# cobot6: (x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg); gantry: 2 or 3 linear axes in mm.
Pose = tuple[float, ...]

POSE_LENGTHS: dict[str, frozenset[int]] = {
    "cobot6": frozenset({6}),
    "gantry": frozenset({2, 3}),
}


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class MachineConfig(_Strict):
    kind: Literal["vmc", "lathe"]
    controller: str
    # Must be stated explicitly per machine: does the workholding have its own
    # "released" sensor? If not, the controller uses the unclamp_fallback procedure.
    clamp_released_sensor: StrictBool


class UnclampFallback(_Strict):
    """Used only on machines without a clamp-released sensor.

    Sequence (implemented in the controller): gripper holds the part -> unclamp ->
    wait release_wait_s -> force-limited pull to pull_pose -> force limit trips => SAFE.
    """

    release_wait_s: float = Field(gt=0)
    pull_pose: str
    pull_force_limit_n: float = Field(gt=0)


class RobotConfig(_Strict):
    kind: Literal["cobot6", "gantry"]


class Timeouts(_Strict):
    pick_raw: float = Field(gt=0)
    load: float = Field(gt=0)
    clamp: float = Field(gt=0)
    retreat: float = Field(gt=0)
    machining: float = Field(gt=0)
    unclamp: float = Field(gt=0)
    unload: float = Field(gt=0)
    place_done: float = Field(gt=0)
    door: float = Field(gt=0)


class Pins(_Strict):
    door_open_out: str
    door_close_out: str
    door_open_in: str
    door_closed_in: str
    clamp_out: str
    clamped_in: str
    unclamped_in: str | None  # required key; null only when clamp_released_sensor is false
    cycle_start_out: str
    cycle_done_in: str
    feed_hold_out: str
    alarm_in: str
    estop_ok_in: str
    guard_closed_in: str


class WatchmanConfig(_Strict):
    sample_hz: float = Field(gt=0)
    stale_after_s: float = Field(gt=0)
    baseline_window_s: float = Field(gt=0)
    tool_break_current_ratio: float = Field(gt=0, lt=1)
    overload_current_a: float = Field(gt=0)
    vibration_rms_max_g: float = Field(gt=0)
    chip_drift_ratio: float = Field(gt=1)


class CellConfig(_Strict):
    cell_id: str
    machine: MachineConfig
    robot: RobotConfig
    poses: dict[str, Pose]
    machine_zone_poses: frozenset[str]
    timeouts_s: Timeouts
    pins: Pins
    watchman: WatchmanConfig
    # Required when machine.clamp_released_sensor is false, forbidden when true.
    unclamp_fallback: UnclampFallback | None = None

    @field_validator("poses")
    @classmethod
    def _all_poses_present(cls, v: dict[str, Pose]) -> dict[str, Pose]:
        missing = REQUIRED_POSES - v.keys()
        if missing:
            raise ValueError(f"missing poses: {sorted(missing)}")
        return v

    @model_validator(mode="after")
    def _zone_poses_exist(self) -> CellConfig:
        unknown = self.machine_zone_poses - self.poses.keys()
        if unknown:
            raise ValueError(f"machine_zone_poses not in poses: {sorted(unknown)}")
        if not self.machine_zone_poses:
            raise ValueError("machine_zone_poses must not be empty")
        return self

    @model_validator(mode="after")
    def _pose_shape_matches_robot(self) -> CellConfig:
        allowed = POSE_LENGTHS[self.robot.kind]
        bad = sorted(n for n, p in self.poses.items() if len(p) not in allowed)
        if bad:
            raise ValueError(
                f"poses {bad} have wrong number of axes for robot kind {self.robot.kind!r} "
                f"(expected {sorted(allowed)})"
            )
        return self

    @model_validator(mode="after")
    def _clamp_release_check_is_explicit(self) -> CellConfig:
        has_sensor = self.machine.clamp_released_sensor
        if has_sensor:
            if self.pins.unclamped_in is None:
                raise ValueError("clamp_released_sensor is true but pins.unclamped_in is null")
            if self.unclamp_fallback is not None:
                raise ValueError(
                    "unclamp_fallback must be absent when clamp_released_sensor is true"
                )
        else:
            if self.pins.unclamped_in is not None:
                raise ValueError("clamp_released_sensor is false but pins.unclamped_in is set")
            if self.unclamp_fallback is None:
                raise ValueError("clamp_released_sensor is false: unclamp_fallback is required")
            if self.unclamp_fallback.pull_pose not in self.poses:
                raise ValueError(
                    f"unclamp_fallback.pull_pose {self.unclamp_fallback.pull_pose!r} not in poses"
                )
        return self


def load_cell_config(path: str | Path) -> CellConfig:
    with open(path) as f:
        raw = yaml.safe_load(f)
    return CellConfig.model_validate(raw)
