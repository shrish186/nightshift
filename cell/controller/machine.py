"""Cell controller: an explicit, table-driven state machine.

SAFETY-RELEVANT: changes here need plan mode and human review (see CLAUDE.md).

How it runs: step() is called every tick. Each tick, in this order:
  1. SAFE is latched: nothing happens until a human reset().
  2. Cell-wide checks: e-stop, guard, CNC alarm, robot status, sensor plausibility,
     and any request_safe() from the watchman. Any failure -> SAFE.
  3. The current state's guards (checked every tick, including the first).
  4. The current state's timeout.
  5. The current state's steps (moves, commands, waits, requirements), in order.
  6. When all steps are done, transition to the state's next state.
Any exception anywhere -> SAFE. A state missing from the table -> SAFE.

This controller is not the safety circuit. E-stop and guarding are a certified
safety relay in hardware (see CLAUDE.md). SAFE is the software's reaction on top.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TypeAlias

from cell.clock import Clock
from cell.config import CellConfig, ProgramConfig
from cell.controller.alerts import Alert, Alerter
from cell.controller.plausibility import IoPlausibilityMonitor
from cell.controller.states import State
from cell.drivers.cnc_io import CncIo
from cell.drivers.gripper import Gripper
from cell.drivers.robot import Robot, RobotStatus
from cell.drivers.safety_in import SafetyInputs
from cell.log import get_logger, log_transition

log = get_logger("cell.controller")


@dataclass(frozen=True)
class Ctx:
    """Everything a guard or step may look at or command."""

    cfg: CellConfig
    robot: Robot
    gripper: Gripper
    cnc: CncIo
    # True once the worst-case stroke time has passed since the last command on a pair.
    stroke_done: Callable[[str], bool]


# --- steps: the fixed sequence inside one state ---


@dataclass(frozen=True)
class Move:
    pose: str


@dataclass(frozen=True)
class MoveLimited:
    pose: str
    force_limit_n: float


@dataclass(frozen=True)
class Command:
    label: str
    fn: Callable[[Ctx], None]


@dataclass(frozen=True)
class WaitUntil:
    label: str
    cond: Callable[[Ctx], bool]


@dataclass(frozen=True)
class Dwell:
    seconds: float


@dataclass(frozen=True)
class Require:
    """Checked once, in order. False -> SAFE."""

    label: str
    cond: Callable[[Ctx], bool]


Step: TypeAlias = Move | MoveLimited | Command | WaitUntil | Dwell | Require

# A guard returns None when OK, or the reason it failed.
Guard: TypeAlias = Callable[[Ctx], str | None]


@dataclass(frozen=True)
class StateSpec:
    steps: Callable[[CellConfig], tuple[Step, ...]]
    guards: tuple[Guard, ...]
    timeout: Callable[[CellConfig, ProgramConfig], float]  # (cell config, active program)
    next_states: frozenset[State]
    next: Callable[[CellConfig], State]


# --- guards ---


def arm_may_be_inside(c: Ctx) -> str | None:
    """While the arm is in or entering the machine: door confirmed open, spindle stopped."""
    if not c.cnc.door_open():
        return "door not confirmed open"
    if c.cnc.door_closed():
        return "door reads both open and closed"
    if c.cnc.cycle_running():
        return "spindle running"
    return None


def spindle_stopped(c: Ctx) -> str | None:
    return "spindle running" if c.cnc.cycle_running() else None


def arm_outside_machine(c: Ctx) -> str | None:
    pose = c.robot.at_pose()
    if pose is None:
        return "arm position unknown"
    if pose in c.cfg.machine_zone_poses:
        return f"arm inside machine at {pose}"
    return None


def machine_ready_to_cut(c: Ctx) -> str | None:
    if not c.cnc.door_closed():
        return "door not confirmed closed"
    if not c.cnc.clamped():
        return "part not confirmed clamped"
    if not c.cnc.part_present():
        return "part not seated"
    return arm_outside_machine(c)


def no_feed_hold(c: Ctx) -> str | None:
    """A feed hold the cell did not command (operator at the machine, or the machine
    itself) freezes the cut. Stop and tell someone instead of waiting for the timeout.
    Watchman feed holds arrive with request_safe() first, so they are named as such."""
    if c.cnc.feed_hold_active():
        return "feed hold during machining (not commanded by the cell)"
    return None


# --- the table ---


def _has_part(c: Ctx) -> bool:
    return c.gripper.has_part()


def _clamp_confirmed(c: Ctx) -> bool:
    if not c.cnc.clamped():
        return False
    # With a released sensor fitted, it must agree (plausibility also checks this).
    return not (c.cfg.machine.clamp_released_sensor and c.cnc.unclamped())


def _door_open_confirmed(c: Ctx) -> bool:
    return c.cnc.door_open() and not c.cnc.door_closed()


def _open_door_steps(_: CellConfig) -> tuple[Step, ...]:
    return (
        Command("open door", lambda c: c.cnc.open_door()),
        WaitUntil("door open", _door_open_confirmed),
        WaitUntil("door worst-case stroke time", lambda c: c.stroke_done("door")),
    )


def _fallback_steps(cfg: CellConfig) -> tuple[Step, ...]:
    fb = cfg.unclamp_fallback
    if fb is None:  # config validation makes this impossible; fail closed anyway
        return (Require("unclamp_fallback configured", lambda c: False),)
    return (
        Command("unclamp", lambda c: c.cnc.unclamp()),
        Dwell(fb.release_wait_s),
        MoveLimited(fb.pull_pose, fb.pull_force_limit_n),
    )


def _fixed(*steps: Step) -> Callable[[CellConfig], tuple[Step, ...]]:
    return lambda cfg: steps


def _goto(state: State) -> tuple[frozenset[State], Callable[[CellConfig], State]]:
    return frozenset({state}), lambda cfg: state


def _spec(
    steps: Callable[[CellConfig], tuple[Step, ...]],
    guards: tuple[Guard, ...],
    timeout: Callable[[CellConfig, ProgramConfig], float],
    to: tuple[frozenset[State], Callable[[CellConfig], State]],
) -> StateSpec:
    return StateSpec(steps, guards, timeout, to[0], to[1])


_INSIDE = (arm_may_be_inside,)

STATE_TABLE: dict[State, StateSpec] = {
    State.PICK_RAW: _spec(
        _fixed(
            Command("gripper open", lambda c: c.gripper.open()),
            WaitUntil("gripper open", lambda c: c.gripper.is_open()),
            Move("above_raw_tray"),
            Move("pick_raw"),
            Command("gripper close", lambda c: c.gripper.close()),
            WaitUntil("gripper closed", lambda c: c.gripper.is_closed()),
            Require("raw part in gripper", _has_part),
            Move("above_raw_tray"),
        ),
        (),
        lambda cfg, p: cfg.timeouts_s.pick_raw,
        _goto(State.OPEN_DOOR_LOAD),
    ),
    State.OPEN_DOOR_LOAD: _spec(
        _open_door_steps,
        (spindle_stopped,),
        lambda cfg, p: cfg.timeouts_s.door,
        _goto(State.LOAD),
    ),
    State.LOAD: _spec(
        _fixed(
            Require("raw part in gripper", _has_part),
            Require("fixture empty", lambda c: not c.cnc.part_present()),
            Move("above_fixture"),
            Move("load"),
            Require("raw part in gripper", _has_part),
        ),
        _INSIDE,
        lambda cfg, p: cfg.timeouts_s.load,
        _goto(State.CLAMP),
    ),
    State.CLAMP: _spec(
        _fixed(
            WaitUntil("part seated in fixture", lambda c: c.cnc.part_present()),
            Command("clamp", lambda c: c.cnc.clamp()),
            WaitUntil("clamped", _clamp_confirmed),
            WaitUntil("clamp worst-case stroke time", lambda c: c.stroke_done("clamp")),
            Command("gripper open", lambda c: c.gripper.open()),
            WaitUntil("gripper open", lambda c: c.gripper.is_open()),
        ),
        _INSIDE,
        lambda cfg, p: cfg.timeouts_s.clamp,
        _goto(State.RETREAT),
    ),
    State.RETREAT: _spec(
        _fixed(Move("above_fixture"), Move("clear_of_machine")),
        _INSIDE,
        lambda cfg, p: cfg.timeouts_s.retreat,
        _goto(State.CLOSE_DOOR),
    ),
    State.CLOSE_DOOR: _spec(
        _fixed(
            Command("close door", lambda c: c.cnc.close_door()),
            WaitUntil("door closed", lambda c: c.cnc.door_closed() and not c.cnc.door_open()),
        ),
        (arm_outside_machine,),
        lambda cfg, p: cfg.timeouts_s.door,
        _goto(State.MACHINING),
    ),
    State.MACHINING: _spec(
        _fixed(
            Command("cycle start", lambda c: c.cnc.cycle_start()),
            WaitUntil("cycle done", lambda c: c.cnc.cycle_done()),
        ),
        (no_feed_hold, machine_ready_to_cut),
        lambda cfg, p: p.expected_cycle_s * cfg.timeouts_s.machining_factor,
        _goto(State.OPEN_DOOR_UNLOAD),
    ),
    State.OPEN_DOOR_UNLOAD: _spec(
        _open_door_steps,
        (spindle_stopped,),
        lambda cfg, p: cfg.timeouts_s.door,
        _goto(State.ENTER_UNLOAD),
    ),
    State.ENTER_UNLOAD: _spec(
        _fixed(
            # Closed jaws driven onto the finished part would crash into it.
            Require("gripper open", lambda c: c.gripper.is_open()),
            Move("above_fixture"),
            Move("load"),
            Command("gripper close", lambda c: c.gripper.close()),
            WaitUntil("gripper closed", lambda c: c.gripper.is_closed()),
            Require("finished part in gripper", _has_part),
        ),
        _INSIDE,
        lambda cfg, p: cfg.timeouts_s.unload,
        # The only way out is through a release check. Which one is fixed by config.
        (
            frozenset({State.UNCLAMP, State.UNCLAMP_FALLBACK}),
            lambda cfg: (
                State.UNCLAMP if cfg.machine.clamp_released_sensor else State.UNCLAMP_FALLBACK
            ),
        ),
    ),
    State.UNCLAMP: _spec(
        _fixed(
            Command("unclamp", lambda c: c.cnc.unclamp()),
            WaitUntil("unclamp confirmed", lambda c: c.cnc.unclamped() and not c.cnc.clamped()),
            WaitUntil("unclamp worst-case stroke time", lambda c: c.stroke_done("clamp")),
            Move("above_fixture"),
        ),
        _INSIDE,
        lambda cfg, p: cfg.timeouts_s.unclamp,
        _goto(State.EXIT),
    ),
    State.UNCLAMP_FALLBACK: _spec(
        _fallback_steps,
        _INSIDE,
        lambda cfg, p: cfg.timeouts_s.unclamp,
        _goto(State.EXIT),
    ),
    State.EXIT: _spec(
        _fixed(Move("clear_of_machine")),
        _INSIDE,
        lambda cfg, p: cfg.timeouts_s.retreat,
        _goto(State.PLACE_DONE),
    ),
    State.PLACE_DONE: _spec(
        _fixed(
            Require("finished part in gripper", _has_part),
            Move("above_done_tray"),
            Move("place_done"),
            Command("gripper open", lambda c: c.gripper.open()),
            WaitUntil("gripper open", lambda c: c.gripper.is_open()),
            Move("above_done_tray"),
            Move("home"),
        ),
        (),
        lambda cfg, p: cfg.timeouts_s.place_done,
        _goto(State.IDLE),
    ),
}


@dataclass(frozen=True)
class Transition:
    ts: float
    src: State
    dst: State
    reason: str


class _GoSafe(Exception):
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


_STOPPED_STATUSES = (RobotStatus.FAULT, RobotStatus.STOPPED, RobotStatus.FORCE_LIMIT)


class CellController:
    def __init__(
        self,
        cfg: CellConfig,
        clock: Clock,
        robot: Robot,
        gripper: Gripper,
        cnc: CncIo,
        safety: SafetyInputs,
        alerter: Alerter,
    ) -> None:
        self._cfg = cfg
        self._clock = clock
        self._safety = safety
        self._alerter = alerter
        self._plausibility = IoPlausibilityMonitor(cnc, clock, cfg)
        # Door/clamp commands go through the monitor's wrapper so it knows what was asked.
        self._ctx = Ctx(
            cfg, robot, gripper, self._plausibility.cnc, self._plausibility.stroke_time_elapsed
        )
        self.state = State.IDLE
        self.cycles_completed = 0
        self.last_safe_reason = ""
        self.history: list[Transition] = []
        # Extra checks reset() must pass, e.g. the watchman's health. Each returns None
        # when OK, or why a reset must be refused.
        self.reset_checks: list[Callable[[], str | None]] = []
        self._start_requested = False
        self.program: str | None = None  # active CNC program, set by start_cycle()
        self._safe_requested: str | None = None
        self._entered_at = clock.now()
        self._steps: tuple[Step, ...] = ()
        self._step_idx = 0
        self._step_issued = False
        self._step_started_at = 0.0

    # --- public API ---
    def start_cycle(self, program: str) -> bool:
        """Request one cycle of a configured CNC program. Only accepted in IDLE."""
        if self.state is not State.IDLE or self._start_requested:
            return False
        if program not in self._cfg.programs:
            log.info(
                "start refused",
                extra={
                    "fields": {
                        "event": "start_refused",
                        "reason": f"unknown program {program!r}",
                        "ts": self._clock.now(),
                        "cell_id": self._cfg.cell_id,
                    }
                },
            )
            return False
        self.program = program
        self._start_requested = True
        return True

    def request_safe(self, reason: str) -> None:
        """Go to SAFE on the next step(). Used by the watchman. Always accepted."""
        if self._safe_requested is None:
            self._safe_requested = reason

    def step(self) -> None:
        try:
            self._tick()
        except _GoSafe as e:
            self._enter_safe(e.reason)
        except Exception as e:  # any unexpected error -> SAFE
            self._enter_safe(f"exception: {type(e).__name__}: {e}")

    def reset(self, operator: str) -> str | None:
        """Human-initiated exit from SAFE to IDLE. Returns None if accepted, else why not.

        Before calling, a person must have: made the cell physically safe, released the
        e-stop, closed the guard, cleared the alarm/feed hold at the machine, and jogged
        the arm to a known pose outside the machine using the robot's own pendant.
        """
        if self.state is not State.SAFE:
            return f"not in SAFE (state {self.state.value})"
        c = self._ctx
        blockers = []
        if not self._safety.estop_ok():
            blockers.append("e-stop not released")
        if not self._safety.guard_closed():
            blockers.append("guard not closed")
        if c.cnc.alarm():
            blockers.append("CNC alarm not cleared")
        if c.cnc.feed_hold_active():
            blockers.append("feed hold not cleared at the machine")
        if not (c.cnc.door_closed() and not c.cnc.door_open()):
            blockers.append("door not confirmed closed")
        for check in self.reset_checks:
            why = check()
            if why is not None:
                blockers.append(why)
        # The person has checked the cell: accept current readings as the new baseline.
        self._plausibility.rebaseline()
        faults = self._plausibility.check()
        if faults:
            blockers.append(f"sensor plausibility: {[f.value for f in faults]}")
        # Check where the arm is BEFORE unlatching it: never release a robot that is
        # inside the machine or at an unknown position.
        where = arm_outside_machine(c)
        if where is not None:
            blockers.append(where)
        if not blockers:
            c.robot.reset()  # only now, with every check passed, unlatch the robot
            if c.robot.status() is not RobotStatus.IDLE:
                blockers.append(f"robot not ready ({c.robot.status().value})")
                c.robot.stop()  # re-latch the robot while we stay in SAFE
        if blockers:
            reason = "; ".join(blockers)
            log.info(
                "reset refused",
                extra={
                    "fields": {
                        "event": "reset_refused",
                        "operator": operator,
                        "reason": reason,
                        "ts": self._clock.now(),
                        "cell_id": self._cfg.cell_id,
                    }
                },
            )
            return reason
        self._safe_requested = None
        self._start_requested = False
        self._transition(State.IDLE, f"reset by {operator}")
        return None

    # --- internals ---
    def _tick(self) -> None:
        if self.state is State.SAFE:
            return
        if self._safe_requested is not None:
            raise _GoSafe(f"requested: {self._safe_requested}")
        self._cell_checks()

        if self.state is State.IDLE:
            if self._start_requested:
                self._start_requested = False
                self._transition(State.PICK_RAW, "cycle started")
            return

        spec = STATE_TABLE.get(self.state)
        if spec is None:
            raise _GoSafe(f"no spec for state {self.state}")
        for guard in spec.guards:
            why = guard(self._ctx)
            if why is not None:
                raise _GoSafe(f"{self.state.value} guard {guard.__name__}: {why}")
        if self.program is None:
            raise _GoSafe(f"{self.state.value}: no active program")
        timeout = spec.timeout(self._cfg, self._cfg.programs[self.program])
        if self._clock.now() - self._entered_at > timeout:
            raise _GoSafe(f"{self.state.value} timeout: {self._describe_step()}")

        while self._step_idx < len(self._steps):
            if not self._run_step(self._steps[self._step_idx]):
                return
            self._step_idx += 1
            self._step_issued = False

        nxt = spec.next(self._cfg)
        if nxt not in spec.next_states:
            raise _GoSafe(f"{self.state.value} chose illegal next state {nxt}")
        if nxt is State.IDLE:
            self.cycles_completed += 1
        self._transition(nxt, f"{self.state.value} complete")

    def _cell_checks(self) -> None:
        c = self._ctx
        if not self._safety.estop_ok():
            raise _GoSafe("e-stop")
        if not self._safety.guard_closed():
            raise _GoSafe("guard open")
        if c.cnc.alarm():
            raise _GoSafe("CNC alarm")
        status = c.robot.status()
        if status in _STOPPED_STATUSES:
            raise _GoSafe(f"robot {status.value}")
        faults = self._plausibility.check()
        if faults:
            raise _GoSafe(f"sensor plausibility: {[f.value for f in faults]}")

    def _run_step(self, step: Step) -> bool:
        """Advance one step. True when it is complete."""
        c = self._ctx
        first = not self._step_issued
        if first:
            self._step_issued = True
            self._step_started_at = self._clock.now()
        if isinstance(step, Move):
            if first:
                c.robot.move_to(step.pose)
            return c.robot.at_pose() == step.pose
        if isinstance(step, MoveLimited):
            if first:
                c.robot.move_to_limited(step.pose, step.force_limit_n)
            return c.robot.at_pose() == step.pose
        if isinstance(step, Command):
            step.fn(c)
            # A command can itself reveal a fault (e.g. plausibility rule d). Check now,
            # before any later step in this same tick acts on the result.
            self._cell_checks()
            return True
        if isinstance(step, WaitUntil):
            return step.cond(c)
        if isinstance(step, Dwell):
            return self._clock.now() - self._step_started_at >= step.seconds
        if isinstance(step, Require):
            if not step.cond(c):
                raise _GoSafe(f"{self.state.value} requirement failed: {step.label}")
            return True
        raise _GoSafe(f"unknown step {step!r}")

    def _describe_step(self) -> str:
        if self._step_idx >= len(self._steps):
            return "finishing"
        s = self._steps[self._step_idx]
        return getattr(s, "label", None) or getattr(s, "pose", None) or type(s).__name__

    def _transition(self, dst: State, reason: str) -> None:
        src, now = self.state, self._clock.now()
        self.history.append(Transition(now, src, dst, reason))
        log_transition(log, src.value, dst.value, reason, now, self._cfg.cell_id)
        self.state = dst
        self._entered_at = now
        spec = STATE_TABLE.get(dst)
        self._steps = spec.steps(self._cfg) if spec is not None else ()
        self._step_idx = 0
        self._step_issued = False

    def _enter_safe(self, reason: str) -> None:
        if self.state is State.SAFE:
            return
        c, errors = self._ctx, []
        # Each stop action is attempted even if another one fails.
        for label, action in (("robot stop", c.robot.stop), ("feed hold", c.cnc.feed_hold)):
            try:
                action()
            except Exception as e:
                errors.append(f"{label} failed: {type(e).__name__}: {e}")
        if errors:
            reason = f"{reason} [{'; '.join(errors)}]"
        prior = self.state
        self.last_safe_reason = reason
        self._start_requested = False
        self._transition(State.SAFE, reason)
        try:
            self._alerter.alert(Alert(self._clock.now(), self._cfg.cell_id, prior.value, reason))
        except Exception as e:  # alerting must never keep the cell out of SAFE
            log.error("alert failed", extra={"fields": {"event": "alert_failed", "error": str(e)}})
