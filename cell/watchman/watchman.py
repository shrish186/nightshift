"""Watchman runtime. SAFETY-RELEVANT: changes here need human review.

Runs independently of the controller: its own tick(), its own sensor reads. It never
waits for the controller to act on a STOP finding:
  1. cnc.feed_hold() directly (CLAUDE.md: the watchman may do this at any time),
  2. then request_safe() so the controller goes to SAFE and stops the arm,
  3. then it latches until a human reset().
ALERT findings are sent to the alerter, at most once per kind per cut.
Stale sensor data or an error inside the watchman makes it UNHEALTHY (alert, and
health() reports it): no new unattended work starts or loads, the cut in progress
finishes. A node failure never stops the machine by itself.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Protocol

from cell.clock import Clock
from cell.config import CellConfig
from cell.controller.alerts import Alert, Alerter
from cell.drivers.sensors import Sensors
from cell.log import get_logger
from cell.watchman.detector import Detector, Finding, FindingKind, Severity
from cell.watchman.reference import CycleStats, Reference, ReferenceStore

log = get_logger("cell.watchman")

ALERT_STATE = "WATCHMAN"  # Alert.state for watchman alerts (controller alerts carry a state)


class MachineLink(Protocol):
    """What the watchman needs from the machine: CncIo on a robot cell, the node's
    feed-hold relay on a standalone machine (cycle_running unused there)."""

    def feed_hold(self) -> None: ...
    def feed_hold_active(self) -> bool: ...
    def cycle_running(self) -> bool: ...


@dataclass
class _Recording:
    program: str
    tool: str
    operator: str
    cycles: list[CycleStats]


class Watchman:
    def __init__(
        self,
        cfg: CellConfig,
        clock: Clock,
        sensors: Sensors,
        cnc: MachineLink,
        request_safe: Callable[[str], None],
        alerter: Alerter,
        store: ReferenceStore | None = None,
    ) -> None:
        self._cfg = cfg
        self._clock = clock
        self._sensors = sensors
        self._cnc = cnc
        self._request_safe = request_safe
        self._alerter = alerter
        self._detector = Detector(cfg.watchman, start=clock.now())
        self.stopped = False
        self.stop_reason = ""
        self._cut = 0
        self._was_cutting = False
        self._alerted: set[tuple[FindingKind, int]] = set()
        self._store = store if store is not None else ReferenceStore(None)
        self.program: str | None = None
        self.supervised = False
        self._findings_in_cut: dict[int, bool] = {}
        self._last_complete: tuple[int, CycleStats] | None = None
        self._confirmed_cuts: set[int] = set()
        self._recording: _Recording | None = None
        self._error: str | None = None

    def tick(self) -> list[Finding]:
        now = self._clock.now()
        try:
            frame = self._sensors.read()
            # Standalone (cut_source "current"): no cut signal; the detector finds the
            # spindle segment and load phase from current.
            signal: bool | None = None
            if self._cfg.watchman.cut_source == "cnc":
                signal = self._cnc.cycle_running() and not self._cnc.feed_hold_active()
            self._error = None
            done_before = self._detector.cuts_completed
            findings = self._detector.update(frame, now, signal)
            cutting = self._detector.cutting
            if cutting and not self._was_cutting:
                self._cut += 1
            self._was_cutting = cutting
            if findings:
                self._findings_in_cut[self._cut] = True
            if self._detector.cuts_completed > done_before and self._detector.last_cut:
                self._last_complete = (self._cut, self._detector.last_cut)
        except Exception as e:
            self._error = f"watchman error: {type(e).__name__}: {e}"
            findings = [Finding(FindingKind.STALE, Severity.ALERT, self._error)]
        for f in findings:
            if f.severity is Severity.STOP:
                self._stop(f, now)
            else:
                self._alert(f, now)
        return findings

    # --- programs and references ---
    def _tool(self, program: str) -> str:
        return self._cfg.programs[program].tool

    def prepare(self, program: str, supervised: bool) -> str | None:
        """Controller start check. Selects the program's reference. None = OK to start.

        Unattended runs need a confirmed reference for this program + tool; supervised
        runs don't (and are the only way to record one)."""
        if program not in self._cfg.programs:
            return f"unknown program {program!r}"
        if self._recording is not None and not supervised:
            return "reference recording in progress: runs must be supervised"
        ref = self._store.get(program, self._tool(program))
        if ref is None and not supervised:
            return (
                f"no confirmed watchman reference for program {program} + tool "
                f"{self._tool(program)}: record one in a supervised run"
            )
        self.program, self.supervised = program, supervised
        self._detector.set_reference(ref)
        return None

    def begin_reference(
        self, program: str, operator: str, fresh_tool_confirmed: bool
    ) -> str | None:
        """Start recording a reference. The operator must confirm the tool is fresh."""
        if program not in self._cfg.programs:
            return f"unknown program {program!r}"
        if not fresh_tool_confirmed:
            return "operator must confirm a fresh tool before recording a reference"
        self._recording = _Recording(program, self._tool(program), operator, [])
        self._log_event("reference_begin", f"{program}/{self._tool(program)} by {operator}")
        return None

    def confirm_cycle_clean(self, operator: str) -> str | None:
        """Operator confirms the last completed cut was clean. None = accepted."""
        rec = self._recording
        if rec is None:
            return "not recording a reference"
        if self._last_complete is None or self._last_complete[0] in self._confirmed_cuts:
            return "no new completed cycle to confirm"
        if self.program != rec.program or not self.supervised:
            return "the last cycle was not a supervised run of the program being recorded"
        cut, stats = self._last_complete
        if self._findings_in_cut.get(cut):
            return "that cycle had watchman findings; it cannot be part of a reference"
        self._confirmed_cuts.add(cut)
        rec.cycles.append(stats)
        if len(rec.cycles) >= self._cfg.watchman.reference_cycles:
            ref = Reference(
                rec.program,
                rec.tool,
                rec.operator,
                True,
                datetime.now(UTC).isoformat(timespec="seconds"),
                tuple(rec.cycles),
            )
            self._store.put(ref)
            self._recording = None
            self._detector.set_reference(ref)
            self._log_event("reference_saved", f"{ref.program}/{ref.tool} confirmed by {operator}")
        return None

    def reject_cycle(self) -> None:
        """Operator says the last cycle was not clean: it is never used."""
        if self._last_complete is not None:
            self._confirmed_cuts.add(self._last_complete[0])

    @property
    def recording_progress(self) -> tuple[int, int] | None:
        if self._recording is None:
            return None
        return len(self._recording.cycles), self._cfg.watchman.reference_cycles

    def reference_for(self, program: str) -> Reference | None:
        return self._store.get(program, self._tool(program))

    def tool_changed(self, tool: str, operator: str) -> list[Reference]:
        """Every reference for this tool is deleted and must be re-recorded."""
        dropped = self._store.drop_tool(tool)
        if self.program is not None and self._tool(self.program) == tool:
            self._detector.set_reference(None)
        if self._recording is not None and self._recording.tool == tool:
            self._recording = None
        self._log_event(
            "tool_changed", f"{tool} by {operator}; dropped {len(dropped)} reference(s)"
        )
        return dropped

    def health(self) -> str | None:
        """For the controller's reset(): None if the watchman can watch, else why not."""
        if self.stopped:
            return f"watchman stop latched ({self.stop_reason}): reset the watchman first"
        if self._error is not None:
            return self._error
        try:
            frame = self._sensors.read()
        except Exception as e:
            return f"watchman: sensor read failed: {type(e).__name__}: {e}"
        now = self._clock.now()
        if frame is None or now - frame.ts >= self._cfg.watchman.stale_after_s:
            return "watchman: no fresh sensor data"
        return None

    def reset(self) -> None:
        """Human-initiated, together with the controller reset. Keeps the learned reference."""
        self.stopped = False
        self.stop_reason = ""

    def _stop(self, f: Finding, now: float) -> None:
        if self.stopped:
            return
        self.stopped = True
        reason = f"watchman: {f.kind.value}: {f.detail}"
        try:
            self._cnc.feed_hold()
        except Exception as e:  # still ask the controller to stop everything
            reason = f"{reason} [feed hold failed: {type(e).__name__}: {e}]"
        self.stop_reason = reason
        self._log("watchman_stop", f, now)
        try:
            self._request_safe(reason)
        except Exception as e:
            log.error(
                "request_safe failed",
                extra={"fields": {"event": "watchman_request_failed", "error": str(e)}},
            )

    def _alert(self, f: Finding, now: float) -> None:
        key = (f.kind, self._cut)
        if key in self._alerted:
            return
        self._alerted.add(key)
        self._log("watchman_alert", f, now)
        try:
            self._alerter.alert(
                Alert(now, self._cfg.cell_id, ALERT_STATE, f"watchman: {f.kind.value}: {f.detail}")
            )
        except Exception as e:
            log.error("alert failed", extra={"fields": {"event": "alert_failed", "error": str(e)}})

    def _log_event(self, event: str, detail: str) -> None:
        log.info(
            event,
            extra={
                "fields": {
                    "event": event,
                    "detail": detail,
                    "ts": self._clock.now(),
                    "cell_id": self._cfg.cell_id,
                }
            },
        )

    def _log(self, event: str, f: Finding, now: float) -> None:
        log.info(
            event,
            extra={
                "fields": {
                    "event": event,
                    "kind": f.kind.value,
                    "severity": f.severity.value,
                    "detail": f.detail,
                    "ts": now,
                    "cell_id": self._cfg.cell_id,
                }
            },
        )
