"""Door and clamp sensor plausibility. SAFETY-RELEVANT: changes here need human review.

Static rules (per sensor pair, e.g. door open + door closed):
  - both True at once                                   -> *_BOTH_ON
  - both False for longer than the plausibility window  -> *_BOTH_OFF_TOO_LONG

Command-aware rules. The controller sends door/clamp commands through `monitor.cnc`,
a thin CncIo wrapper that tells the monitor what was commanded and when:
  (a) the destination sensor confirms faster than min_travel_fraction x measured
      travel: a stuck-on sensor, not a real stroke     -> *_CONFIRMED_TOO_FAST
  (b) the origin sensor has not cleared within the plausibility window after the
      command: stuck on                                 -> *_ORIGIN_STUCK
  (c) a reading changes while no command is in progress: the mechanism moved on its
      own, or a sensor failed                           -> *_UNCOMMANDED_CHANGE
  (d) a command whose destination already reads True, when this monitor never saw
      the mechanism arrive there: nothing proves the reading -> *_CONFIRMED_WITHOUT_TRAVEL

A stuck-on + dead pair (e.g. open stuck on, closed dead) reads perfectly consistent,
so the static rules cannot see it; (a), (c) and (d) do.

rebaseline() is for the human reset only: it accepts the current readings as the new
baseline. The door must read closed at reset (enforced by the controller), so the
door's confirmed position after a reset is "closed" and never a possibly-stuck "open".
The controller calls check() every step and goes to SAFE on any fault.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum

from cell.clock import Clock
from cell.config import CellConfig
from cell.drivers.cnc_io import CncIo


class PlausibilityFault(Enum):
    DOOR_BOTH_ON = "door_both_on"
    DOOR_BOTH_OFF_TOO_LONG = "door_both_off_too_long"
    DOOR_CONFIRMED_TOO_FAST = "door_confirmed_too_fast"
    DOOR_ORIGIN_STUCK = "door_origin_stuck"
    DOOR_UNCOMMANDED_CHANGE = "door_uncommanded_change"
    DOOR_CONFIRMED_WITHOUT_TRAVEL = "door_confirmed_without_travel"
    CLAMP_BOTH_ON = "clamp_both_on"
    CLAMP_BOTH_OFF_TOO_LONG = "clamp_both_off_too_long"
    CLAMP_CONFIRMED_TOO_FAST = "clamp_confirmed_too_fast"
    CLAMP_ORIGIN_STUCK = "clamp_origin_stuck"
    CLAMP_UNCOMMANDED_CHANGE = "clamp_uncommanded_change"
    CLAMP_CONFIRMED_WITHOUT_TRAVEL = "clamp_confirmed_without_travel"


def _f(pair: str, rule: str) -> PlausibilityFault:
    return PlausibilityFault(f"{pair}_{rule}")


@dataclass
class _Command:
    target: str  # "a" (open / clamped) or "b" (closed / released)
    at: float
    motion_expected: bool
    confirmed: bool = False


@dataclass
class _Pair:
    """Sensor a = open/clamped, sensor b = closed/released (None if not fitted)."""

    name: str  # "door" or "clamp"
    read_a: Callable[[], bool]
    read_b: Callable[[], bool] | None
    window_s: float
    travel_s: float
    min_travel_fraction: float
    off_since: float | None = None
    last: tuple[bool, bool | None] | None = None
    confirmed_pos: str | None = None  # where this monitor last saw the mechanism arrive
    cmd: _Command | None = None
    pending: list[PlausibilityFault] = field(default_factory=list)

    def read(self) -> tuple[bool, bool | None]:
        return self.read_a(), (self.read_b() if self.read_b is not None else None)

    def _dest_origin(self, target: str, a: bool, b: bool | None) -> tuple[bool, bool | None]:
        return (a, b) if target == "a" else (b if b is not None else not a, a)

    def command(self, target: str, now: float) -> None:
        a, b = self.read()
        dest, _ = self._dest_origin(target, a, b)
        motion = not dest
        if target == "b" and b is None:
            motion = True  # no released sensor: no destination to confirm or distrust
        elif not motion and self.confirmed_pos != target:
            self.pending.append(_f(self.name, "confirmed_without_travel"))  # rule (d)
        self.cmd = _Command(target, now, motion)

    def check(self, now: float) -> list[PlausibilityFault]:
        a, b = self.read()
        out, self.pending = self.pending, []

        if b is not None:  # static rules need both sensors
            if a and b:
                self.off_since = None
                out.append(_f(self.name, "both_on"))
            elif a or b:
                self.off_since = None
            else:
                self.off_since = now if self.off_since is None else self.off_since
                if now - self.off_since > self.window_s:
                    out.append(_f(self.name, "both_off_too_long"))

        cmd = self.cmd
        if cmd is not None and not cmd.confirmed and cmd.target == "b" and b is None:
            # No released sensor: nothing can confirm "released" (the controller uses
            # the force-limited pull instead). Accept once "clamped" has cleared.
            if not a:
                cmd.confirmed = True
                self.confirmed_pos = None
        elif cmd is not None and not cmd.confirmed:
            dest, origin = self._dest_origin(cmd.target, a, b)
            if dest and not (b is not None and origin):
                cmd.confirmed = True
                self.confirmed_pos = cmd.target
                if cmd.motion_expected and now - cmd.at < self.min_travel_fraction * self.travel_s:
                    out.append(_f(self.name, "confirmed_too_fast"))  # rule (a)
            elif b is not None and cmd.motion_expected and origin and now - cmd.at > self.window_s:
                out.append(_f(self.name, "origin_stuck"))  # rule (b)
        elif self.last is not None and (a, b) != self.last:
            out.append(_f(self.name, "uncommanded_change"))  # rule (c)
        self.last = (a, b)
        return out

    def stroke_time_elapsed(self, now: float, max_travel_fraction: float) -> bool:
        """True once the worst-case stroke time has passed since the last command."""
        if self.cmd is None or not self.cmd.motion_expected:
            return True
        return now - self.cmd.at >= max_travel_fraction * self.travel_s

    def rebaseline(self) -> None:
        a, b = self.read()
        self.last = (a, b)
        self.off_since = None
        self.cmd = None
        self.pending = []
        # Only a consistent "b" reading (closed / released) is accepted as a known position.
        self.confirmed_pos = "b" if (b is not None and b and not a) else None


class CommandedCnc:
    """CncIo that reports door/clamp commands to the monitor, then forwards them."""

    def __init__(self, inner: CncIo, monitor: IoPlausibilityMonitor) -> None:
        self._inner = inner
        self._monitor = monitor

    def open_door(self) -> None:
        self._monitor.note_command("door", "a")
        self._inner.open_door()

    def close_door(self) -> None:
        self._monitor.note_command("door", "b")
        self._inner.close_door()

    def clamp(self) -> None:
        self._monitor.note_command("clamp", "a")
        self._inner.clamp()

    def unclamp(self) -> None:
        self._monitor.note_command("clamp", "b")
        self._inner.unclamp()

    def cycle_start(self) -> None:
        self._inner.cycle_start()

    def feed_hold(self) -> None:
        self._inner.feed_hold()

    def door_open(self) -> bool:
        return self._inner.door_open()

    def door_closed(self) -> bool:
        return self._inner.door_closed()

    def clamped(self) -> bool:
        return self._inner.clamped()

    def unclamped(self) -> bool:
        return self._inner.unclamped()

    def part_present(self) -> bool:
        return self._inner.part_present()

    def cycle_running(self) -> bool:
        return self._inner.cycle_running()

    def cycle_done(self) -> bool:
        return self._inner.cycle_done()

    def feed_hold_active(self) -> bool:
        return self._inner.feed_hold_active()

    def alarm(self) -> bool:
        return self._inner.alarm()


class IoPlausibilityMonitor:
    def __init__(self, cnc: CncIo, clock: Clock, cfg: CellConfig) -> None:
        self._clock = clock
        p = cfg.io_plausibility
        self._max_travel_fraction = p.max_travel_fraction
        self.cnc = CommandedCnc(cnc, self)  # send door/clamp commands through this
        self._pairs = {
            "door": _Pair(
                "door",
                cnc.door_open,
                cnc.door_closed,
                p.door_plausibility_window_s,
                p.door_travel_s,
                p.min_travel_fraction,
            ),
            "clamp": _Pair(
                "clamp",
                cnc.clamped,
                cnc.unclamped if cfg.machine.clamp_released_sensor else None,
                p.clamp_plausibility_window_s,
                p.clamp_travel_s,
                p.min_travel_fraction,
            ),
        }

    def note_command(self, pair: str, target: str) -> None:
        self._pairs[pair].command(target, self._clock.now())

    def stroke_time_elapsed(self, pair: str) -> bool:
        return self._pairs[pair].stroke_time_elapsed(self._clock.now(), self._max_travel_fraction)

    def check(self) -> list[PlausibilityFault]:
        now = self._clock.now()
        return [f for pair in self._pairs.values() for f in pair.check(now)]

    def rebaseline(self) -> None:
        for pair in self._pairs.values():
            pair.rebaseline()
