"""Uploading event batches to the cloud. Outbound HTTPS only: the hub never listens."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from typing import Protocol
from urllib.parse import urlparse

from telemetry.queue import Event


class UploadError(Exception):
    """The batch may or may not have reached the cloud. Retry later; ids make it safe."""


@dataclass(frozen=True)
class UploadResult:
    accepted_ids: list[str]  # ids the cloud confirms it has stored (deduped by id)


class Uploader(Protocol):
    def upload(self, batch: list[Event]) -> UploadResult: ...


class HttpUploader:
    """POST {"events": [...]} with a bearer token; expects {"accepted": [ids]}."""

    def __init__(self, url: str, token: str, timeout_s: float = 10.0) -> None:
        parsed = urlparse(url)
        local = parsed.hostname in ("localhost", "127.0.0.1", "::1")
        if parsed.scheme != "https" and not (parsed.scheme == "http" and local):
            raise ValueError("telemetry endpoint must be https (http only for localhost)")
        if not token:
            raise ValueError("telemetry token is empty")
        self._url = url
        self._token = token
        self._timeout = timeout_s

    @classmethod
    def from_env(cls, url: str, token_env: str, timeout_s: float = 10.0) -> HttpUploader:
        """Token comes from the environment (.env via docker-compose env_file), never config."""
        token = os.environ.get(token_env, "")
        if not token:
            raise ValueError(f"set {token_env} in the hub's .env file")
        return cls(url, token, timeout_s)

    def upload(self, batch: list[Event]) -> UploadResult:
        body = json.dumps(
            {"events": [asdict(e) | {"priority": int(e.priority)} for e in batch]}
        ).encode()
        req = urllib.request.Request(
            self._url,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {self._token}"},
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                data = json.loads(resp.read() or b"{}")
        except urllib.error.HTTPError as e:
            raise UploadError(f"cloud returned HTTP {e.code}") from None
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise UploadError(f"network error: {type(e).__name__}") from None
        except json.JSONDecodeError:
            raise UploadError("cloud returned invalid JSON") from None
        accepted = data.get("accepted")
        if not isinstance(accepted, list) or not all(isinstance(i, str) for i in accepted):
            raise UploadError("cloud reply missing 'accepted' ids")
        return UploadResult(accepted)
