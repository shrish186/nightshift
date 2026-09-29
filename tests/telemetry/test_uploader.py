from __future__ import annotations

import json
import threading
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any

import pytest

from cell.clock import SimClock
from cell.controller.alerts import Alert
from telemetry.agent import TelemetryAgent, TelemetryAlerter
from telemetry.queue import Event, EventQueue, Priority
from telemetry.uploader import HttpUploader, UploadError

TOKEN = "test-token-not-a-secret"


class _Server:
    def __init__(self) -> None:
        self.mode = "ok"
        self.requests: list[dict[str, Any]] = []
        srv = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a: Any) -> None:
                pass

            def do_POST(self) -> None:
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                srv.requests.append({"auth": self.headers.get("Authorization"), "body": body})
                if srv.mode == "500":
                    self.send_response(500)
                    self.end_headers()
                    return
                reply: Any = {"accepted": [e["id"] for e in body["events"]]}
                if srv.mode == "garbage":
                    reply = {"nope": 1}
                data = json.dumps(reply).encode()
                self.send_response(200)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        self.httpd = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_port}/v1/events"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()


@pytest.fixture
def server() -> Iterator[_Server]:
    s = _Server()
    yield s
    s.httpd.shutdown()


def _events(n: int) -> list[Event]:
    return [Event(f"e{i}", float(i), "features", Priority.BULK, {"i": i}) for i in range(n)]


def test_upload_returns_confirmed_ids_and_sends_token(server: _Server) -> None:
    result = HttpUploader(server.url, TOKEN).upload(_events(3))
    assert result.accepted_ids == ["e0", "e1", "e2"]
    assert server.requests[0]["auth"] == f"Bearer {TOKEN}"
    assert server.requests[0]["body"]["events"][1]["payload"] == {"i": 1}


@pytest.mark.parametrize("mode", ["500", "garbage"])
def test_bad_replies_are_retryable_errors(server: _Server, mode: str) -> None:
    server.mode = mode
    with pytest.raises(UploadError):
        HttpUploader(server.url, TOKEN).upload(_events(1))


def test_unreachable_is_a_retryable_error_without_the_token() -> None:
    with pytest.raises(UploadError) as e:
        HttpUploader("http://127.0.0.1:9/v1/events", TOKEN, timeout_s=0.5).upload(_events(1))
    assert TOKEN not in str(e.value)


def test_plain_http_refused_except_localhost() -> None:
    with pytest.raises(ValueError, match="https"):
        HttpUploader("http://api.example.com/v1/events", TOKEN)
    HttpUploader("https://api.example.com/v1/events", TOKEN)


def test_token_comes_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("NIGHTSHIFT_TELEMETRY_TOKEN", raising=False)
    with pytest.raises(ValueError, match="NIGHTSHIFT_TELEMETRY_TOKEN") as e:
        HttpUploader.from_env("https://api.example.com/v1/events", "NIGHTSHIFT_TELEMETRY_TOKEN")
    monkeypatch.setenv("NIGHTSHIFT_TELEMETRY_TOKEN", TOKEN)
    HttpUploader.from_env("https://api.example.com/v1/events", "NIGHTSHIFT_TELEMETRY_TOKEN")
    assert TOKEN not in str(e.value)


def test_end_to_end_with_real_http(server: _Server, tmp_path: Any) -> None:
    clock = SimClock()
    agent = TelemetryAgent(
        EventQueue(tmp_path / "q.db", max_bulk=1000),
        HttpUploader(server.url, TOKEN),
        clock,
        batch_size=50,
        backoff_initial_s=1,
        backoff_max_s=10,
    )
    TelemetryAlerter(agent).alert(Alert(1.0, "sim-01", "MACHINING", "watchman: tool_break"))
    for i in range(120):
        agent.record("features", {"i": i}, Priority.BULK)
    while agent.sync_once():
        pass
    sent = [e for r in server.requests for e in r["body"]["events"]]
    assert len(sent) == 121 and sent[0]["kind"] == "alert"  # alert goes first
    assert agent.queue.stats()["pending"] == 0
