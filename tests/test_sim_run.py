"""Smoke tests for the sim.run command-line tool (non-live mode)."""

from __future__ import annotations

from pathlib import Path

import pytest

from cell.sim.run import main, parse_fault


def test_clean_run_completes_all_cycles(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["--cycles", "3", "--no-live", "--cycle-s", "10", "--seed", "1"])
    out = capsys.readouterr().out
    assert code == 0
    assert "unattended cycles: 3/3 completed" in out
    assert "reference: recorded 5 supervised cycles" in out
    assert "final state: IDLE" in out
    assert "unsafe events: 0" in out


def test_fault_run_ends_safe_with_reason(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(
        ["--cycles", "3", "--no-live", "--cycle-s", "10", "--fault", "TOOL_BREAK@MACHINING+4#2"]
    )
    out = capsys.readouterr().out
    assert code == 0  # going SAFE is the correct outcome, not an error
    assert "unattended cycles: 1/3 completed" in out
    assert "final state: SAFE" in out and "watchman: tool_break" in out
    assert "unsafe events: 0" in out


def test_no_release_sensor_machine_runs(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["--cycles", "2", "--no-live", "--cycle-s", "8", "--no-release-sensor"])
    assert code == 0
    assert "unattended cycles: 2/2 completed" in capsys.readouterr().out


def test_random_faults_run(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(["--cycles", "3", "--no-live", "--cycle-s", "10", "--random-faults", "2",
                 "--seed", "7"])  # fmt: skip
    assert code == 0
    assert "unsafe events: 0" in capsys.readouterr().out


@pytest.mark.parametrize(
    "spec", ["NOPE@MACHINING", "TOOL_BREAK@NOWHERE", "TOOL_BREAK@MACHINING+x", "TOOL_BREAK"]
)
def test_bad_fault_spec_rejected(spec: str) -> None:
    with pytest.raises(ValueError):
        parse_fault(spec)


def test_fault_spec_parses() -> None:
    f = parse_fault("door_sensors_stuck_open@open_door_load+1.5#3")
    assert f.fault.name == "DOOR_SENSORS_STUCK_OPEN"
    assert f.state.name == "OPEN_DOOR_LOAD" and f.delay_s == 1.5 and f.cycle == 3


def test_logs_go_to_file_not_console(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    import json

    log = tmp_path / "run.jsonl"
    assert main(["--cycles", "1", "--no-live", "--cycle-s", "6", "--log", str(log)]) == 0
    out = capsys.readouterr()
    assert '"event": "transition"' not in out.out + out.err
    events = [json.loads(line) for line in log.read_text().splitlines()]
    assert any(e["event"] == "reference_saved" for e in events)
    assert sum(e["event"] == "transition" for e in events) >= 13
