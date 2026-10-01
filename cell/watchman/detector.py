"""Watchman fault detection: a pure function of the samples it is fed.

SAFETY-RELEVANT: changes here need human review.

Graded responses:
  STOP  : TOOL_BREAK, OVERLOAD (current or vibration).
  ALERT : STALE sensor data (watchman unhealthy: blocks new unattended work, the cut
          in progress continues).
  ALERT : CHIP_BUILDUP (load climbing within one cut) and TOOL_WEAR (load climbing
          cycle over cycle), escalating to STOP only past their separate hard ratios.

Every detector either needs a condition to hold for a confirm window or works on
window means, so a single noisy sample can never stop the cell.

Levels are relative, not absolute, wherever possible:
  - tool break: vs. this cut's settled mean current (breaks mid-cut), and this cut's
    early window mean vs. the reference cycles (tool already broken when the cut starts)
  - chip buildup: recent window mean vs. this cut's early window mean
  - tool wear: each cut's early window mean vs. the confirmed reference for this
    program + tool (see reference.py; never learned automatically)
Alert limits come from the reference's recorded spread when there is one, bounded by
the config alert ratios; stop ratios are the fixed config values.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum

from cell.config import WatchmanConfig
from cell.drivers.sensors import SensorFrame
from cell.watchman.hardstop import (
    HS_OVERLOAD,
    HS_SENSOR_FAULT,
    HS_TOOL_BREAK,
    HardStop,
    HardStopConfig,
)
from cell.watchman.reference import CycleStats, Reference

_EPS = 1e-9


class FindingKind(Enum):
    TOOL_BREAK = "tool_break"
    OVERLOAD = "overload"
    STALE = "stale"
    CHIP_BUILDUP = "chip_buildup"
    TOOL_WEAR = "tool_wear"
    SENSOR_FAULT = "sensor_fault"  # invalid reading, ADC clipping, or flat vibration


class Severity(Enum):
    ALERT = "alert"
    STOP = "stop"


@dataclass(frozen=True)
class Finding:
    kind: FindingKind
    severity: Severity
    detail: str


def _mean(xs: list[float] | deque[float]) -> float:
    return sum(xs) / len(xs)


class Detector:
    def __init__(
        self, cfg: WatchmanConfig, start: float, reference: Reference | None = None
    ) -> None:
        self._cfg = cfg
        self._start = start
        self._last_ts: float | None = None
        self._was_cutting = False
        self.set_reference(reference)
        self.last_cut: CycleStats | None = None  # stats of the most recent complete cut
        self.cuts_completed = 0
        self._hs = HardStop(HardStopConfig.from_watchman(cfg))
        self._new_cut(start)

    def set_reference(self, reference: Reference | None) -> None:
        """The confirmed reference for the active program + tool, or None. Without one,
        the reference-based checks (tool wear, tool broken before the cut) cannot run."""
        self._ref = reference
        self._limits = reference.limits(self._cfg) if reference is not None else None

    def _new_cut(self, now: float) -> None:
        self._cut_start = now
        self._early: list[float] = []
        self._early_judged = False
        self._chip_max = 1.0
        self._recent: deque[tuple[float, float]] = deque()

    def update(self, frame: SensorFrame | None, now: float, cutting: bool) -> list[Finding]:
        out: list[Finding] = []

        # --- STALE: no fresh frame within stale_after_s (including before the first) ---
        fresh = frame is not None and (self._last_ts is None or frame.ts > self._last_ts)
        if fresh and frame is not None:
            self._last_ts = frame.ts
        age = now - (self._last_ts if self._last_ts is not None else self._start)
        if age >= self._cfg.stale_after_s - _EPS:
            # Not a stop: a dead/offline node makes the watchman unhealthy (it can't
            # watch), which blocks unattended starts and loading. The cut in progress
            # finishes (founder decision 2026-09-29).
            return [
                Finding(FindingKind.STALE, Severity.ALERT, f"no fresh sensor data for {age:.2f}s")
            ]
        if not fresh or frame is None:
            return out

        a, v = frame.spindle_current_a, frame.vibration_rms_g
        c = self._cfg

        # --- hard-stop rules: the SAME code path as the node firmware (hardstop.py is the
        # parity-tested reference for node/lib/hardstop). Every fresh frame goes through
        # it so its timers see continuous time; the cut boundary comes from `cutting`.
        ev = self._hs.step(round(now * 1000), a, v, False, 1 if cutting else 0)
        if ev & HS_SENSOR_FAULT:
            out.append(Finding(FindingKind.SENSOR_FAULT, Severity.ALERT, f"current {a} vib {v}"))

        # --- cut boundaries (for the adaptive windows below) ---
        if cutting and not self._was_cutting:
            self._new_cut(now)
        if not cutting and self._was_cutting:
            out += self._end_of_cut()
        self._was_cutting = cutting
        if not cutting:
            return out

        if ev & HS_OVERLOAD:
            out.append(
                Finding(FindingKind.OVERLOAD, Severity.STOP, f"current {a:.1f}A vib {v:.2f}g")
            )
        if ev & HS_TOOL_BREAK:
            out.append(
                Finding(
                    FindingKind.TOOL_BREAK,
                    Severity.STOP,
                    f"current {a:.1f}A vs cut mean {self._hs.cut_mean():.1f}A",
                )
            )

        if now - self._cut_start < c.cut_settle_s - _EPS:
            return out

        # --- CHIP_BUILDUP: recent window mean vs. this cut's early window mean ---
        settled_at = self._cut_start + c.cut_settle_s
        if now < settled_at + c.chip_window_s:
            self._early.append(a)
        elif not self._early_judged:
            # --- TOOL_BREAK before the cut: early level vs. the good-tool reference ---
            self._early_judged = True
            if self._early and self._ref is not None:
                ratio = _mean(self._early) / self._ref.level_mean
                if ratio < c.tool_break_current_ratio:
                    out.append(
                        Finding(
                            FindingKind.TOOL_BREAK,
                            Severity.STOP,
                            f"cut load x{ratio:.2f} of reference from the start",
                        )
                    )
        self._recent.append((now, a))
        while self._recent and self._recent[0][0] <= now - c.chip_window_s:
            self._recent.popleft()
        if now >= settled_at + 2 * c.chip_window_s - _EPS and self._early:
            ratio = _mean([x for _, x in self._recent]) / _mean(self._early)
            self._chip_max = max(self._chip_max, ratio)
            chip_alert = self._limits.chip_alert if self._limits else c.chip_alert_ratio
            if ratio >= c.chip_stop_ratio:
                out.append(
                    Finding(FindingKind.CHIP_BUILDUP, Severity.STOP, f"load x{ratio:.2f} in cut")
                )
            elif ratio >= chip_alert:
                out.append(
                    Finding(FindingKind.CHIP_BUILDUP, Severity.ALERT, f"load x{ratio:.2f} in cut")
                )
        return out

    def _end_of_cut(self) -> list[Finding]:
        """Record this cut's stats; TOOL_WEAR: its early level vs. the reference."""
        c = self._cfg
        complete = self._cut_start + c.cut_settle_s + c.chip_window_s
        if not self._early or self._last_ts is None or self._last_ts < complete:
            return []  # cut too short to judge
        level = _mean(self._early)
        self.last_cut = CycleStats(level, self._chip_max)
        self.cuts_completed += 1
        if self._ref is None or self._limits is None:
            return []
        ratio = level / self._ref.level_mean
        detail = f"load x{ratio:.2f} vs reference"
        if ratio >= c.wear_stop_ratio:
            return [Finding(FindingKind.TOOL_WEAR, Severity.STOP, detail)]
        if ratio >= self._limits.wear_alert:
            return [Finding(FindingKind.TOOL_WEAR, Severity.ALERT, detail)]
        return []
