"""Telemetry agent: offline for hours, flapping network, restart mid-sync.
Nothing is lost (except bulk evicted by the disk cap) and nothing is double-counted."""

from __future__ import annotations

from pathlib import Path

from hypothesis import given, settings
from hypothesis import strategies as st

from cell.clock import SimClock
from telemetry.agent import TelemetryAgent
from telemetry.queue import EventQueue, Priority
from tests.telemetry.fakes import FakeCloud, FakeUploader


def make(path: Path, clock: SimClock, up: FakeUploader, **kw: float) -> TelemetryAgent:
    return TelemetryAgent(
        EventQueue(path, max_bulk=int(kw.get("max_bulk", 100_000))),
        up,
        clock,
        batch_size=int(kw.get("batch_size", 200)),
        backoff_initial_s=kw.get("backoff_initial_s", 1.0),
        backoff_max_s=kw.get("backoff_max_s", 300.0),
        seed=0,
    )


def drain(agent: TelemetryAgent, clock: SimClock, seconds: float, step: float = 1.0) -> None:
    for _ in range(int(seconds / step)):
        agent.sync_once()
        clock.advance(step)


def test_happy_path_delivers_everything_once(tmp_path: Path) -> None:
    clock, cloud = SimClock(), FakeCloud()
    agent = make(tmp_path / "q.db", clock, FakeUploader(cloud))
    ids = [agent.record("features", {"i": i}) for i in range(1000)]
    drain(agent, clock, 20)
    assert set(cloud.received) == set(ids) and cloud.deliveries == len(ids)
    assert agent.queue.stats()["pending"] == 0


def test_six_hours_offline_then_everything_arrives(tmp_path: Path) -> None:
    clock, cloud = SimClock(), FakeCloud()
    up = FakeUploader(cloud)
    up.online = False
    agent = make(tmp_path / "q.db", clock, up)
    ids = []
    for s in range(6 * 3600):  # 1 event/s for 6 h, an alert every 10 min
        ids.append(agent.record("features", {"s": s}))
        if s % 600 == 0:
            ids.append(agent.record("alert", {"s": s}, priority=Priority.CRITICAL))
        agent.sync_once()
        clock.advance(1.0)
    # backoff is capped: roughly one attempt per backoff_max, not one per second
    assert up.calls <= 6 * 3600 / 300 + 20
    up.online = True
    drain(agent, clock, 600)
    assert set(cloud.received) == set(ids)
    assert cloud.deliveries == len(ids)  # nothing sent twice


def test_flapping_network_no_loss_no_duplicates_counted(tmp_path: Path) -> None:
    clock, cloud = SimClock(), FakeCloud()
    up = FakeUploader(cloud, seed=3)
    up.fail_rate = 0.6
    agent = make(tmp_path / "q.db", clock, up, backoff_max_s=20)
    ids = [agent.record("features", {"i": i}) for i in range(3000)]
    drain(agent, clock, 3600)
    assert set(cloud.received) == set(ids)
    assert len(cloud.order) == len(ids)


def test_restart_mid_sync_after_server_stored_batch(tmp_path: Path) -> None:
    """The cloud stores a batch but the ack is lost, then the hub restarts. The batch is
    re-sent (at-least-once) and the cloud's dedupe by id keeps it counted once."""
    clock, cloud = SimClock(), FakeCloud()
    up = FakeUploader(cloud)
    path = tmp_path / "q.db"
    agent = make(path, clock, up, batch_size=50)
    ids = [agent.record("alert", {"i": i}, priority=Priority.CRITICAL) for i in range(120)]
    up.lose_ack = True
    agent.sync_once()  # cloud got 50, hub never heard back
    agent.queue.close()  # hub restarts
    up.lose_ack = False
    agent2 = make(path, clock, up, batch_size=50)
    drain(agent2, clock, 60)
    assert set(cloud.received) == set(ids)
    assert len(cloud.order) == 120  # each counted once
    assert cloud.deliveries == 170  # the 50 were re-sent, as designed


def test_partial_accept_only_marks_accepted(tmp_path: Path) -> None:
    clock, cloud = SimClock(), FakeCloud()
    up = FakeUploader(cloud)
    up.partial = 30
    agent = make(tmp_path / "q.db", clock, up, batch_size=100)
    [agent.record("features", {"i": i}) for i in range(100)]
    agent.sync_once()
    assert agent.queue.stats()["pending"] == 70


def test_backoff_resets_after_success(tmp_path: Path) -> None:
    clock, cloud = SimClock(), FakeCloud()
    up = FakeUploader(cloud)
    up.online = False
    agent = make(tmp_path / "q.db", clock, up)
    agent.record("features", {})
    drain(agent, clock, 1000)
    assert agent.backoff_s >= 100
    up.online = True
    drain(agent, clock, 400)
    assert agent.backoff_s == 0 and agent.queue.stats()["pending"] == 0


@settings(max_examples=60, deadline=None)
@given(
    ops=st.lists(
        st.sampled_from(["record", "record_alert", "sync", "net_down", "net_up", "restart",
                         "lose_ack", "tick"]),
        min_size=1,
        max_size=120,
    )
)  # fmt: skip
def test_random_interleavings_lose_nothing(tmp_path_factory: object, ops: list[str]) -> None:
    import tempfile

    path = Path(tempfile.mkdtemp()) / "q.db"
    clock, cloud = SimClock(), FakeCloud()
    up = FakeUploader(cloud)
    agent = make(path, clock, up, backoff_max_s=5)
    ids: list[str] = []
    for op in ops:
        if op == "record":
            ids.append(agent.record("features", {}))
        elif op == "record_alert":
            ids.append(agent.record("alert", {}, priority=Priority.CRITICAL))
        elif op == "sync":
            agent.sync_once()
        elif op == "net_down":
            up.online = False
        elif op == "net_up":
            up.online, up.lose_ack = True, False
        elif op == "lose_ack":
            up.lose_ack = True
        elif op == "restart":
            agent.queue.close()
            agent = make(path, clock, up, backoff_max_s=5)
        clock.advance(1.0)
    up.online, up.lose_ack, up.fail_rate = True, False, 0.0
    drain(agent, clock, 200)
    assert set(cloud.received) == set(ids)
    assert len(cloud.order) == len(set(ids))
