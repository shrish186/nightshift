"""Watchman fault detection: a pure function of the samples it is fed.

SAFETY-RELEVANT: changes here need human review.

Graded responses:
  STOP  : TOOL_BREAK, OVERLOAD (current or vibration), STALE sensor data.
  ALERT : CHIP_BUILDUP (load climbing within one cut) and TOOL_WEAR (load climbing
          cycle over cycle), escalating to STOP only past their separate hard ratios.

Every detector either needs a condition to hold for a confirm window or works on
window means, so a single noisy sample can never stop the cell.

Levels are relative, not absolute, wherever possible:
  - tool break: vs. this cut's settled mean current
  - chip buildup: recent window mean vs. this cut's early window mean
  - tool wear: each cut's early window mean vs. the reference cycles' mean
ASSUMPTION: the tool is good for the first `reference_cycles` cuts after start.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum

from cell.config import WatchmanConfig
from cell.drivers.sensors import SensorFrame

_EPS = 1e-9
_MIN_SETTLED_SAMPLES = 5


class FindingKind(Enum):
    TOOL_BREAK = "tool_break"
    OVERLOAD = "overload"
    STALE = "stale"
    CHIP_BUILDUP = "chip_buildup"
    TOOL_WEAR = "tool_wear"


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
    def __init__(self, cfg: WatchmanConfig, start: float) -> None:
        self._cfg = cfg
        self._start = start
        self._last_ts: float | None = None
        self._was_cutting = False
        self._reference: list[float] = []
        self._new_cut(start)

    def _new_cut(self, now: float) -> None:
        self._cut_start = now
        self._settled_sum = 0.0
        self._settled_n = 0
        self._below_since: float | None = None
        self._over_since: float | None = None
        self._early: list[float] = []
        self._recent: deque[tuple[float, float]] = deque()

    def update(self, frame: SensorFrame | None, now: float, cutting: bool) -> list[Finding]:
        out: list[Finding] = []

        # --- STALE: no fresh frame within stale_after_s (including before the first) ---
        fresh = frame is not None and (self._last_ts is None or frame.ts > self._last_ts)
        if fresh and frame is not None:
            self._last_ts = frame.ts
        age = now - (self._last_ts if self._last_ts is not None else self._start)
        if age >= self._cfg.stale_after_s - _EPS:
            return [
                Finding(FindingKind.STALE, Severity.STOP, f"no fresh sensor data for {age:.2f}s")
            ]
        if not fresh or frame is None:
            return out

        # --- cut boundaries ---
        if cutting and not self._was_cutting:
            self._new_cut(now)
        if not cutting and self._was_cutting:
            out += self._end_of_cut()
        self._was_cutting = cutting
        if not cutting:
            return out

        a, v = frame.spindle_current_a, frame.vibration_rms_g
        c = self._cfg

        # --- OVERLOAD: current or vibration above limit for the confirm window ---
        over = a > c.overload_current_a or v > c.vibration_rms_max_g
        if over:
            self._over_since = now if self._over_since is None else self._over_since
            if now - self._over_since >= c.overload_confirm_s - _EPS:
                out.append(
                    Finding(FindingKind.OVERLOAD, Severity.STOP, f"current {a:.1f}A vib {v:.2f}g")
                )
        else:
            self._over_since = None

        if now - self._cut_start < c.cut_settle_s - _EPS:
            return out

        # --- TOOL_BREAK: current collapses vs. this cut's settled mean, for confirm window ---
        below = False
        if self._settled_n >= _MIN_SETTLED_SAMPLES:
            mean = self._settled_sum / self._settled_n
            below = a < c.tool_break_current_ratio * mean
            if below:
                self._below_since = now if self._below_since is None else self._below_since
                if now - self._below_since >= c.tool_break_confirm_s - _EPS:
                    out.append(
                        Finding(
                            FindingKind.TOOL_BREAK,
                            Severity.STOP,
                            f"current {a:.1f}A vs cut mean {mean:.1f}A",
                        )
                    )
        if not below:
            self._below_since = None
            if not over:
                self._settled_sum += a
                self._settled_n += 1

        # --- CHIP_BUILDUP: recent window mean vs. this cut's early window mean ---
        settled_at = self._cut_start + c.cut_settle_s
        if now < settled_at + c.chip_window_s:
            self._early.append(a)
        self._recent.append((now, a))
        while self._recent and self._recent[0][0] <= now - c.chip_window_s:
            self._recent.popleft()
        if now >= settled_at + 2 * c.chip_window_s - _EPS and self._early:
            ratio = _mean([x for _, x in self._recent]) / _mean(self._early)
            if ratio >= c.chip_stop_ratio:
                out.append(
                    Finding(FindingKind.CHIP_BUILDUP, Severity.STOP, f"load x{ratio:.2f} in cut")
                )
            elif ratio >= c.chip_alert_ratio:
                out.append(
                    Finding(FindingKind.CHIP_BUILDUP, Severity.ALERT, f"load x{ratio:.2f} in cut")
                )
        return out

    def _end_of_cut(self) -> list[Finding]:
        """TOOL_WEAR: this cut's early-window level vs. the reference cuts."""
        c = self._cfg
        complete = self._cut_start + c.cut_settle_s + c.chip_window_s
        if not self._early or self._last_ts is None or self._last_ts < complete:
            return []  # cut too short to judge
        level = _mean(self._early)
        if len(self._reference) < c.reference_cycles:
            self._reference.append(level)
            return []
        ratio = level / _mean(self._reference)
        if ratio >= c.wear_stop_ratio:
            return [
                Finding(FindingKind.TOOL_WEAR, Severity.STOP, f"load x{ratio:.2f} vs reference")
            ]
        if ratio >= c.wear_alert_ratio:
            return [
                Finding(FindingKind.TOOL_WEAR, Severity.ALERT, f"load x{ratio:.2f} vs reference")
            ]
        return []
