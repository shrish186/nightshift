"""Watchman runtime. SAFETY-RELEVANT: changes here need human review.

Runs independently of the controller: its own tick(), its own sensor reads. It never
waits for the controller to act on a STOP finding:
  1. cnc.feed_hold() directly (CLAUDE.md: the watchman may do this at any time),
  2. then request_safe() so the controller goes to SAFE and stops the arm,
  3. then it latches until a human reset().
ALERT findings are sent to the alerter, at most once per kind per cut.
Any error inside the watchman itself is treated as a STOP (it can no longer watch).
"""

from __future__ import annotations

from collections.abc import Callable

from cell.clock import Clock
from cell.config import CellConfig
from cell.controller.alerts import Alert, Alerter
from cell.drivers.cnc_io import CncIo
from cell.drivers.sensors import Sensors
from cell.log import get_logger
from cell.watchman.detector import Detector, Finding, FindingKind, Severity

log = get_logger("cell.watchman")

ALERT_STATE = "WATCHMAN"  # Alert.state for watchman alerts (controller alerts carry a state)


class Watchman:
    def __init__(
        self,
        cfg: CellConfig,
        clock: Clock,
        sensors: Sensors,
        cnc: CncIo,
        request_safe: Callable[[str], None],
        alerter: Alerter,
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

    def tick(self) -> list[Finding]:
        now = self._clock.now()
        try:
            frame = self._sensors.read()
            cutting = self._cnc.cycle_running() and not self._cnc.feed_hold_active()
            if cutting and not self._was_cutting:
                self._cut += 1
            self._was_cutting = cutting
            findings = self._detector.update(frame, now, cutting)
        except Exception as e:
            findings = [
                Finding(
                    FindingKind.STALE, Severity.STOP, f"watchman error: {type(e).__name__}: {e}"
                )
            ]
        for f in findings:
            if f.severity is Severity.STOP:
                self._stop(f, now)
            else:
                self._alert(f, now)
        return findings

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
