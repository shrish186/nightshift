"""Hardware safety inputs. SAFETY-RELEVANT: changes here need human review.

READ ONLY. The e-stop chain and guard interlocks are wired in hardware and stop the
robot and machine on their own. Software only reads them so it can react (go to SAFE,
alert). There is intentionally no method to set, override, or simulate these from
the controller.

The circuit itself is a certified safety relay (see CLAUDE.md). These reads are
monitoring contacts from that relay, not the safety function.

FAIL-SAFE WIRING. ASSUMPTION / VERIFY ON HARDWARE:
Both inputs are true only while the signal is actively present. A broken wire,
loose connector or loss of power reads False (e-stop NOT ok, guard NOT closed).
HW to confirm the relay's monitoring outputs behave this way on the real panel.
"""

from __future__ import annotations

from typing import Protocol


class SafetyInputs(Protocol):
    def estop_ok(self) -> bool:
        """True only when the e-stop chain is healthy (no e-stop pressed)."""

    def guard_closed(self) -> bool:
        """True only when all guard doors/light curtains report closed/clear."""

    def zone_interlock_ok(self) -> bool:
        """True only when the safety relay reports the door-zone interlock healthy.

        The interlock itself (safety-rated door switch -> safety relay -> robot safety
        stop on entry to the machine zone) is hardware; this is its monitoring contact.
        """
