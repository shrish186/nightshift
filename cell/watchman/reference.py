"""Watchman references: what "normal" looks like for one program + tool.

SAFETY-RELEVANT: changes here need human review.

A reference is only ever recorded during a supervised run, after an operator confirms
the tool is fresh, from N cycles the operator confirms were clean. It is never learned
automatically. A tool change deletes every reference for that tool.

Alert limits come from the reference's own spread (mean + k sigma, with a minimum
margin so a very tight spread can't produce a limit that noise trips). The config
alert ratios are the loosest a derived limit may ever be; stop ratios stay fixed.
"""

from __future__ import annotations

import json
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path

from cell.config import WatchmanConfig


@dataclass(frozen=True)
class CycleStats:
    level: float  # mean cutting current over the cut's early window
    chip_max: float  # highest recent/early ratio seen in the cut (1.0 if the cut was short)


@dataclass(frozen=True)
class Limits:
    wear_alert: float
    chip_alert: float


@dataclass(frozen=True)
class Reference:
    program: str
    tool: str
    operator: str
    fresh_tool_confirmed: bool
    recorded_at: str  # ISO timestamp, wall clock
    cycles: tuple[CycleStats, ...]

    @property
    def level_mean(self) -> float:
        return statistics.fmean(c.level for c in self.cycles)

    def limits(self, cfg: WatchmanConfig) -> Limits:
        levels = [c.level for c in self.cycles]
        chips = [c.chip_max for c in self.cycles]
        k, floor = cfg.derived_limit_sigmas, cfg.derived_min_margin
        level_cv = statistics.stdev(levels) / statistics.fmean(levels)
        wear = 1.0 + max(k * level_cv, floor)
        chip = statistics.fmean(chips) + max(k * statistics.stdev(chips), floor)
        return Limits(
            wear_alert=min(cfg.wear_alert_ratio, wear),
            chip_alert=min(cfg.chip_alert_ratio, chip),
        )


class ReferenceStore:
    """References keyed by (program, tool). JSON file on disk; path None = memory only."""

    def __init__(self, path: str | Path | None) -> None:
        self._path = Path(path) if path is not None else None
        self._refs: dict[tuple[str, str], Reference] = {}
        if self._path is not None and self._path.exists():
            raw = json.loads(self._path.read_text())
            for r in raw["references"]:
                ref = Reference(
                    r["program"],
                    r["tool"],
                    r["operator"],
                    r["fresh_tool_confirmed"],
                    r["recorded_at"],
                    tuple(CycleStats(**c) for c in r["cycles"]),
                )
                self._refs[(ref.program, ref.tool)] = ref

    def get(self, program: str, tool: str) -> Reference | None:
        return self._refs.get((program, tool))

    def put(self, ref: Reference) -> None:
        if not ref.fresh_tool_confirmed:
            raise ValueError("a reference needs operator confirmation of a fresh tool")
        self._refs[(ref.program, ref.tool)] = ref
        self._save()

    def drop_tool(self, tool: str) -> list[Reference]:
        dropped = [r for key, r in self._refs.items() if key[1] == tool]
        for r in dropped:
            del self._refs[(r.program, r.tool)]
        self._save()
        return dropped

    def _save(self) -> None:
        if self._path is None:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        data = {"references": [asdict(r) for r in self._refs.values()]}
        tmp = self._path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2))
        tmp.replace(self._path)  # atomic: never a half-written reference file
