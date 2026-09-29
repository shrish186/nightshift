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

    Unclamp (in the controller): gripper holds the part -> unclamp -> wait release_wait_s
    -> force-limited pull to pull_pose -> force limit trips => SAFE.

    Clamp verification: with one clamp sensor, a clamp that failed to close plus a
    late short on that sensor looks like a clean clamp. So before the gripper lets go,
    the robot tugs the part toward tug_pose (a few mm) with tug_force_n; the clamp must
    resist. ASSUMPTION / VERIFY ON HARDWARE: the arm/gantry can do a force-limited move
    that reports "resisted" without latching, and tug_force_n is well below the clamp's
    holding force but above the part's weight and friction.
    """

    release_wait_s: float = Field(gt=0)
    pull_pose: str
    pull_force_limit_n: float = Field(gt=0)
    tug_pose: str
    tug_force_n: float = Field(gt=0)


class ProgramConfig(_Strict):
    """One CNC program the cell runs. ASSUMPTION / VERIFY ON HARDWARE: expected_cycle_s
    is measured on the machine for this program, not estimated."""

    expected_cycle_s: float = Field(gt=0)
    tool: str  # the tool (or tool set) this program cuts with; watchman references key on it


class RobotConfig(_Strict):
    kind: Literal["cobot6", "gantry"]


class Timeouts(_Strict):
    pick_raw: float = Field(gt=0)
    load: float = Field(gt=0)
    clamp: float = Field(gt=0)
    retreat: float = Field(gt=0)
    # MACHINING timeout = active program's expected_cycle_s x machining_factor.
    machining_factor: float = Field(gt=1)
    unclamp: float = Field(gt=0)
    unload: float = Field(gt=0)
    place_done: float = Field(gt=0)
    door: float = Field(gt=0)


class IoPlausibility(_Strict):
    """Sensor-pair plausibility timing. ASSUMPTION / VERIFY ON HARDWARE.

    *_travel_s must be the real stroke time *measured on this machine*. Each
    *_plausibility_window_s is travel + margin: how long both sensors in a pair may
    read off before it counts as a fault. Separate from timeouts_s.door/clamp, which
    bound how long the controller waits for a commanded move.
    """

    door_travel_s: float = Field(gt=0)
    door_plausibility_window_s: float = Field(gt=0)
    clamp_travel_s: float = Field(gt=0)
    clamp_plausibility_window_s: float = Field(gt=0)
    # A commanded stroke that confirms faster than this fraction of measured travel is a
    # stuck-on sensor, not a real stroke.
    min_travel_fraction: float = Field(gt=0, lt=1)
    # After a stroke confirms, the arm still waits until max_travel_fraction x measured
    # travel has passed since the command before acting on it, so even a sensor that
    # sticks on late in a stroke cannot let the arm move before the stroke could finish.
    max_travel_fraction: float = Field(gt=1)

    @model_validator(mode="after")
    def _window_exceeds_travel(self) -> IoPlausibility:
        if self.door_plausibility_window_s <= self.door_travel_s:
            raise ValueError("door_plausibility_window_s must be > door_travel_s")
        if self.clamp_plausibility_window_s <= self.clamp_travel_s:
            raise ValueError("clamp_plausibility_window_s must be > clamp_travel_s")
        return self


class Pins(_Strict):
    door_open_out: str
    door_close_out: str
    door_open_in: str
    door_closed_in: str
    clamp_out: str
    clamped_in: str
    unclamped_in: str | None  # required key; null only when clamp_released_sensor is false
    part_present_in: str
    cycle_start_out: str
    cycle_done_in: str
    feed_hold_out: str
    alarm_in: str
    estop_ok_in: str
    guard_closed_in: str


class WatchmanConfig(_Strict):
    """Watchman thresholds. ASSUMPTION / VERIFY ON HARDWARE: every value is a placeholder
    until tuned from M1 data per machine, tool and material.

    Graded responses: tool break, overload and stale data stop the cell. Chip buildup
    and tool wear only alert, and stop only past their separate hard *_stop_ratio.
    """

    sample_hz: float = Field(gt=0)
    stale_after_s: float = Field(gt=0)
    cut_settle_s: float = Field(ge=0)  # ignore spindle ramp-up at the start of each cut
    tool_break_current_ratio: float = Field(gt=0, lt=1)
    tool_break_confirm_s: float = Field(gt=0)
    overload_current_a: float = Field(gt=0)
    vibration_rms_max_g: float = Field(gt=0)
    overload_confirm_s: float = Field(gt=0)
    chip_window_s: float = Field(gt=0)
    chip_alert_ratio: float = Field(gt=1)
    chip_stop_ratio: float = Field(gt=1)
    wear_alert_ratio: float = Field(gt=1)
    wear_stop_ratio: float = Field(gt=1)
    # References are recorded per program + tool in a supervised run: an operator
    # confirms a fresh tool, then confirms reference_cycles cycles clean. Never auto-learned.
    reference_cycles: int = Field(ge=3)
    # Alert limits from the reference spread: mean + derived_limit_sigmas x sd, at least
    # derived_min_margin above the mean; never looser than the *_alert_ratio above.
    derived_limit_sigmas: float = Field(gt=0)
    derived_min_margin: float = Field(gt=0)
    reference_store: str  # JSON file of recorded references (under data/, not in git)

    @model_validator(mode="after")
    def _stop_above_alert(self) -> WatchmanConfig:
        if self.chip_stop_ratio <= self.chip_alert_ratio:
            raise ValueError("chip_stop_ratio must be > chip_alert_ratio")
        if self.wear_stop_ratio <= self.wear_alert_ratio:
            raise ValueError("wear_stop_ratio must be > wear_alert_ratio")
        return self


class CellConfig(_Strict):
    cell_id: str
    machine: MachineConfig
    robot: RobotConfig
    poses: dict[str, Pose]
    machine_zone_poses: frozenset[str]
    programs: dict[str, ProgramConfig]
    timeouts_s: Timeouts
    io_plausibility: IoPlausibility
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

    @field_validator("programs")
    @classmethod
    def _at_least_one_program(cls, v: dict[str, ProgramConfig]) -> dict[str, ProgramConfig]:
        if not v:
            raise ValueError("programs must not be empty")
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
            if self.timeouts_s.unclamp <= self.unclamp_fallback.release_wait_s:
                raise ValueError(
                    "timeouts_s.unclamp must be > unclamp_fallback.release_wait_s "
                    "(the wait happens inside the UNCLAMP_FALLBACK state)"
                )
            fb = self.unclamp_fallback
            if fb.tug_pose not in self.poses:
                raise ValueError(f"unclamp_fallback.tug_pose {fb.tug_pose!r} not in poses")
            if fb.tug_pose not in self.machine_zone_poses:
                raise ValueError("unclamp_fallback.tug_pose must be in machine_zone_poses")
            if fb.tug_force_n > fb.pull_force_limit_n:
                raise ValueError("unclamp_fallback.tug_force_n must be <= pull_force_limit_n")
            if self.unclamp_fallback.pull_pose not in self.poses:
                raise ValueError(
                    f"unclamp_fallback.pull_pose {self.unclamp_fallback.pull_pose!r} not in poses"
                )
        return self


def load_cell_config(path: str | Path) -> CellConfig:
    with open(path) as f:
        raw = yaml.safe_load(f)
    return CellConfig.model_validate(raw)
