"""Harness for standalone-machine tests: the watchman fed by StandaloneMachine."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from cell.config import CellConfig, load_cell_config
from cell.controller.alerts import MemoryAlerter
from cell.drivers.sensors import SensorFrame
from cell.drivers.sim.standalone import CyclePlan, StandaloneMachine
from cell.watchman.reference import ReferenceStore
from cell.watchman.watchman import ALERT_STATE, Watchman
from tests.conftest import SIM_CELL

STANDALONE = Path(__file__).resolve().parents[2] / "config" / "watchman-standalone.sim.yaml"
PROGRAM, TOOL, OPERATOR = "P1", "T1", "test-operator"
NOMINAL_CUT_S = 12.0


def standalone_cfg() -> CellConfig:
    import yaml

    raw: dict[str, Any] = load_cell_config(SIM_CELL).model_dump()
    with open(STANDALONE) as f:
        raw["watchman"] = yaml.safe_load(f)["watchman"]
    raw["programs"] = {PROGRAM: {"expected_cycle_s": NOMINAL_CUT_S, "tool": TOOL}}
    return CellConfig.model_validate(raw)


class NodeRelay:
    """The node's feed-hold relay (energised = hold). No CNC I/O on a standalone machine."""

    def __init__(self) -> None:
        self.held = False

    def feed_hold(self) -> None:
        self.held = True

    def feed_hold_active(self) -> bool:
        return self.held

    def cycle_running(self) -> bool:
        return False


class FeedSensors:
    def __init__(self) -> None:
        self.frame: SensorFrame | None = None

    def read(self) -> SensorFrame | None:
        return self.frame


@dataclass
class Standalone:
    machine: StandaloneMachine
    watchman: Watchman
    alerter: MemoryAlerter
    relay: NodeRelay
    stops: list[str] = field(default_factory=list)
    _sensors: FeedSensors = field(default_factory=FeedSensors)

    def run(self, plan: CyclePlan) -> list[str]:
        """One cycle. Returns the stop requests raised during it."""
        before = len(self.stops)

        def on_frame(f: SensorFrame) -> None:
            self._sensors.frame = f
            self.watchman.tick()

        self.machine.run_cycle(plan, on_frame)
        return self.stops[before:]

    @property
    def alerts(self) -> list[str]:
        return [a.reason for a in self.alerter.alerts if a.state == ALERT_STATE]

    def record_reference(self, n: int | None = None) -> None:
        w = self.watchman
        assert w.prepare(PROGRAM, supervised=True) is None
        assert w.begin_reference(PROGRAM, OPERATOR, fresh_tool_confirmed=True) is None
        while w.recording_progress is not None:
            assert self.run(self.machine.plan()) == []
            assert w.confirm_cycle_clean(OPERATOR) is None
        assert w.prepare(PROGRAM, supervised=False) is None


def make_standalone(seed: int = 0, record: bool = True) -> Standalone:
    cfg = standalone_cfg()
    machine = StandaloneMachine(seed=seed, nominal_cut_s=NOMINAL_CUT_S)
    alerter, relay = MemoryAlerter(), NodeRelay()
    s = Standalone(machine, None, alerter, relay)  # type: ignore[arg-type]
    s.watchman = Watchman(
        cfg, machine.clock, s._sensors, relay, s.stops.append, alerter, ReferenceStore(None)
    )
    if record:
        s.record_reference()
    return s
