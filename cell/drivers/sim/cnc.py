"""Simulated CNC machine I/O. Models door, clamp, cycle, feed hold and alarm timing.

The sim keeps *physical truth* (where the door and jaws really are, whether the
spindle is really running) separate from *sensor readings* (what the CncIo inputs
report). Sensor faults change only the readings. The unsafe-event log always checks
physical truth, so a lying sensor cannot hide an unsafe event from the tests.
"""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum

from cell.clock import Clock

# ASSUMPTIONS for a typical auto-door VMC with a pneumatic vise. Verify per machine.
DOOR_S = 2.0
CLAMP_S = 0.5
DEFAULT_CYCLE_S = 60.0
FLICKER_PERIOD_S = 3.0
FLICKER_DROP_S = 0.2
JAW_CONTACT = 0.8  # jaw position (0 open .. 1 closed) above which they grip the part


class CncFault(Enum):
    DOOR_STUCK = "door_stuck"  # door stops part-way on its current/next move
    CLAMP_FAIL = "clamp_fail"  # jaws never close
    ALARM = "alarm"  # machine raises an alarm
    CYCLE_HANG = "cycle_hang"  # cycle never reports done
    CLAMP_STUCK_ON = "clamp_stuck_on"  # jaws stay closed even when unclamp is commanded
    CLAMP_JAM = "clamp_jam"  # jaws jam half-way on their current/next stroke
    DOOR_SENSOR_SHORT = "door_sensor_short"  # both door sensors read true
    CLAMP_SENSOR_SHORT = "clamp_sensor_short"  # both clamp sensors read true
    IO_POWER_LOSS = "io_power_loss"  # I/O loses power: every wire reads de-energised
    # Door starts closing on its own: operator hits the door button, or air pressure drops.
    DOOR_CLOSES_UNCOMMANDED = "door_closes_uncommanded"
    # Stuck-on + dead sensor pairs: readings look perfectly consistent but are wrong.
    DOOR_SENSORS_STUCK_OPEN = "door_sensors_stuck_open"  # open stuck on, closed dead
    DOOR_SENSORS_STUCK_CLOSED = "door_sensors_stuck_closed"  # closed stuck on, open dead
    CLAMP_SENSORS_STUCK_CLAMPED = "clamp_sensors_stuck_clamped"  # clamped on, released dead
    # Fixture seat sensor drops out for 0.2 s every 3 s (coolant, chips, vibration).
    PART_SENSOR_FLICKER = "part_sensor_flicker"
    # Part sits crooked in the fixture: the one there now, or the next one placed.
    PART_MISSEATED = "part_misseated"


class SimCnc:
    def __init__(
        self,
        clock: Clock,
        cycle_s: float = DEFAULT_CYCLE_S,
        clamp_released_sensor: bool = True,
        door_s: float = DOOR_S,
        clamp_s: float = CLAMP_S,
    ) -> None:
        self._clock = clock
        self._door_s = door_s
        self._clamp_s = clamp_s
        self._cycle_s = cycle_s
        self._has_released_sensor = clamp_released_sensor
        self._door_target_open = False
        self._door_done_at = 0.0
        self._door_frozen: str | None = None
        self._door_stick_next = False
        self._clamp_cmd = False
        self._clamp_done_at = 0.0
        self._clamp_fail = False
        self._clamp_started_closed = False  # were the jaws closed when the stroke began
        self._jaws_frozen: str | None = None
        self._in_cycle = False
        self._cycle_end_at = 0.0
        self._cycle_hang = False
        self._done = False
        self._hold = False
        self._alarm = False
        self._sensor_faults: set[CncFault] = set()
        # Physical part in the fixture: "empty", "seated" or "crooked".
        self._fixture = "empty"
        self._misseat_next = False
        self._flicker_from = 0.0
        # Wired by SimCell: True while the arm holds a part at the load pose.
        self.arm_holding_at_load: Callable[[], bool] = lambda: False
        # Wired by SimCell: True if the arm is in the machine envelope.
        self.arm_in_machine: Callable[[], bool] = lambda: False
        self.violations: list[str] = []

    def _update(self) -> None:
        if (
            self._in_cycle
            and not self._hold
            and not self._cycle_hang
            and self._clock.now() >= self._cycle_end_at
        ):
            self._in_cycle = False
            self._done = True

    def _violation(self, what: str) -> None:
        self.violations.append(f"t={self._clock.now():.2f} {what}")

    # --- physical truth (sim only) ---
    def door(self) -> str:
        """Physical door position: "open", "closed" or "moving" (includes stuck half-way)."""
        if self._door_frozen is not None:
            return self._door_frozen
        target = "open" if self._door_target_open else "closed"
        return target if self._clock.now() >= self._door_done_at else "moving"

    def jaw_position(self) -> float:
        """Physical jaw position, 0.0 = fully open .. 1.0 = fully closed on the part."""
        if self._jaws_frozen is not None:
            return {"closed": 1.0, "open": 0.0}.get(self._jaws_frozen, 0.5)
        closing = self._clamp_cmd and not self._clamp_fail
        left = max(0.0, self._clamp_done_at - self._clock.now())
        frac = 1.0 - min(1.0, left / self._clamp_s) if self._clamp_s > 0 else 1.0
        return frac if closing else 1.0 - frac if self._clamp_started_closed else 0.0

    def jaw_contact(self) -> bool:
        """ASSUMPTION: the jaws grip the part in the last 20% of travel toward closed."""
        return self.jaw_position() >= JAW_CONTACT

    def jaws(self) -> str:
        """Physical jaw position: "open", "closed" or "moving"."""
        pos = self.jaw_position()
        return "closed" if pos >= 1.0 else "open" if pos <= 0.0 else "moving"

    def seat(self) -> str:
        """Physical part state in the fixture: "empty", "seated" or "crooked".

        A part still held by the arm at the load pose is already sitting in the fixture.
        """
        if self._fixture != "empty":
            return self._fixture
        if self.arm_holding_at_load():
            return "crooked" if self._misseat_next else "seated"
        return "empty"

    def place_part(self) -> None:
        """Arm released a part at the load pose."""
        self._fixture = "crooked" if self._misseat_next else "seated"
        self._misseat_next = False

    def take_part(self) -> None:
        """Arm gripped the part in the fixture."""
        self._fixture = "empty"

    def fixture_has_part(self) -> bool:
        return self._fixture != "empty"

    def spindle_running(self) -> bool:
        self._update()
        return self._in_cycle

    def spindle_cutting(self) -> bool:
        return self.spindle_running() and not self._hold

    # --- outputs ---
    def _move_door(self, to_open: bool) -> None:
        if self._door_frozen is not None:
            return
        if self.door() == ("open" if to_open else "closed"):
            return  # already there: a real door does not move
        self._door_target_open = to_open
        self._door_done_at = self._clock.now() + self._door_s
        if self._door_stick_next:
            self._door_frozen = "moving"

    def open_door(self) -> None:
        if self.spindle_running():
            self._alarm = True  # real machines refuse; model as alarm
            return
        self._move_door(True)

    def close_door(self) -> None:
        if self.arm_in_machine():
            self._violation("door closed on arm")
        self._move_door(False)

    def clamp(self) -> None:
        self._clamp_started_closed = self.jaw_position() >= 1.0
        self._clamp_cmd = True
        self._clamp_done_at = self._clock.now() + self._clamp_s

    def unclamp(self) -> None:
        if self.spindle_running():
            self._violation("unclamp during cycle")
        self._clamp_started_closed = self.jaw_position() >= 1.0
        self._clamp_cmd = False
        self._clamp_done_at = self._clock.now() + self._clamp_s

    def cycle_start(self) -> None:
        self._update()
        if self.arm_in_machine():
            self._violation("cycle start with arm in machine")
        if self.seat() == "crooked":
            self._violation("cycle start with part not seated")
        # The machine's own interlocks use its own (physical) door and clamp state.
        ready = self.door() == "closed" and self.jaws() == "closed"
        if not ready or self._alarm or self._hold:
            self._alarm = True  # real controls refuse to start; model as alarm
            return
        self._done = False
        self._in_cycle = True
        self._cycle_end_at = self._clock.now() + self._cycle_s

    def feed_hold(self) -> None:
        self._hold = True

    # --- inputs (sensor readings) ---
    @property
    def _powered(self) -> bool:
        return CncFault.IO_POWER_LOSS not in self._sensor_faults

    def door_open(self) -> bool:
        if not self._powered or CncFault.DOOR_SENSORS_STUCK_CLOSED in self._sensor_faults:
            return False
        if CncFault.DOOR_SENSORS_STUCK_OPEN in self._sensor_faults:
            return True
        return CncFault.DOOR_SENSOR_SHORT in self._sensor_faults or self.door() == "open"

    def door_closed(self) -> bool:
        if not self._powered or CncFault.DOOR_SENSORS_STUCK_OPEN in self._sensor_faults:
            return False
        if CncFault.DOOR_SENSORS_STUCK_CLOSED in self._sensor_faults:
            return True
        return CncFault.DOOR_SENSOR_SHORT in self._sensor_faults or self.door() == "closed"

    def clamped(self) -> bool:
        if not self._powered:
            return False
        if CncFault.CLAMP_SENSORS_STUCK_CLAMPED in self._sensor_faults:
            return True
        return CncFault.CLAMP_SENSOR_SHORT in self._sensor_faults or self.jaws() == "closed"

    def unclamped(self) -> bool:
        # No sensor fitted: never reads as released (fail-safe; see CncIo.unclamped).
        if not self._has_released_sensor or not self._powered:
            return False
        if CncFault.CLAMP_SENSORS_STUCK_CLAMPED in self._sensor_faults:
            return False
        return CncFault.CLAMP_SENSOR_SHORT in self._sensor_faults or self.jaws() == "open"

    def part_present(self) -> bool:
        if CncFault.PART_SENSOR_FLICKER in self._sensor_faults:
            since = self._clock.now() - self._flicker_from
            if since % FLICKER_PERIOD_S < FLICKER_DROP_S:
                return False
        return self._powered and self.seat() == "seated"

    # "Bad when true" inputs are wired inverted (see cnc_io.py), so a dead wire reads True.
    def cycle_running(self) -> bool:
        return not self._powered or self.spindle_running()

    def cycle_done(self) -> bool:
        self._update()
        return self._powered and self._done

    def feed_hold_active(self) -> bool:
        return not self._powered or self._hold

    def alarm(self) -> bool:
        return not self._powered or self._alarm

    # --- sim only ---
    def inject(self, fault: CncFault) -> None:
        if fault is CncFault.DOOR_STUCK:
            if self.door() == "moving":
                self._door_frozen = "moving"
            else:
                self._door_stick_next = True
        elif fault is CncFault.CLAMP_FAIL:
            self._clamp_fail = True
        elif fault is CncFault.ALARM:
            self._alarm = True
        elif fault is CncFault.CYCLE_HANG:
            self._cycle_hang = True
        elif fault is CncFault.CLAMP_STUCK_ON:
            self._jaws_frozen = "closed"
        elif fault is CncFault.CLAMP_JAM:
            self._jaws_frozen = "moving"
        elif fault is CncFault.PART_MISSEATED:
            if self._fixture == "seated":
                self._fixture = "crooked"
            else:
                self._misseat_next = True
        elif fault is CncFault.DOOR_CLOSES_UNCOMMANDED:
            # Physical motion only. Not recorded as a controller violation: stopping a
            # closing door on an arm is the door's own hardware protection. What the
            # controller must do is stop sending the arm deeper in.
            self._move_door(False)
        else:
            if fault is CncFault.PART_SENSOR_FLICKER:
                self._flicker_from = self._clock.now()
            self._sensor_faults.add(fault)

    def operator_clear(self) -> None:
        """A human at the machine clears alarm and feed hold."""
        self._alarm = False
        self._hold = False
