"""Hardware safety inputs. SAFETY-RELEVANT: changes here need human review.

READ ONLY. The e-stop chain and guard interlocks are wired in hardware and stop the
robot and machine on their own. Software only reads them so it can react (go to SAFE,
alert). There is intentionally no method to set, override, or simulate these from
the controller.
"""

from __future__ import annotations

from typing import Protocol


class SafetyInputs(Protocol):
    def estop_ok(self) -> bool:
        """True only when the e-stop chain is healthy (no e-stop pressed)."""

    def guard_closed(self) -> bool:
        """True only when all guard doors/light curtains report closed/clear."""
