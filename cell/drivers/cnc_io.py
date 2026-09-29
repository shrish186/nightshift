"""CNC machine I/O interface. SAFETY-RELEVANT: changes here need human review.

This is how the cell and the watchman talk to the machine tool: door, workholding,
cycle start and feed hold, plus the machine's status signals. Commands are
non-blocking; the controller polls the inputs and enforces timeouts.

This is *not* the safety circuit. E-stop and guarding are hardware (see safety_in.py).
feed_hold() is a process stop, not a safety function.

FAIL-SAFE WIRING. ASSUMPTION / VERIFY ON HARDWARE:
Every input must be wired so that a broken wire, loose connector, blown fuse or
loss of I/O power reads as "not safe":
- Permissive inputs (door_open, door_closed, clamped, unclamped, cycle_done) are
  true only while the signal is actively present. A dead wire reads False, e.g.
  door NOT open, so the arm never enters on a broken wire.
- alarm(), feed_hold_active() and cycle_running() mean "bad" when True, so a dead
  wire would read "no alarm" / "spindle stopped". That last one is the permissive
  for the arm entering the machine. Each physical input must therefore be a
  "healthy" signal that is energised when OK ("no alarm", "not held", "spindle
  stopped / cycle idle") and that the driver inverts. A dead wire then reads as
  alarm=True, feed_hold_active=True, cycle_running=True.
- Outputs that let the machine run (cycle_start) must default off on power loss.
HW to confirm the signal polarity for every pin in cells/<cell-id>.yaml on each machine.
"""

from __future__ import annotations

from typing import Protocol


class CncIo(Protocol):
    # --- outputs (commands) ---
    def open_door(self) -> None: ...
    def close_door(self) -> None: ...
    def clamp(self) -> None: ...
    def unclamp(self) -> None: ...
    def cycle_start(self) -> None: ...

    def feed_hold(self) -> None:
        """Pause axis motion. Always accepted. Stays held until a human clears it at the machine."""

    # --- inputs (status) ---
    def door_open(self) -> bool:
        """Door confirmed fully open by its own sensor."""

    def door_closed(self) -> bool:
        """Door confirmed fully closed by its own sensor."""

    def clamped(self) -> bool:
        """Workholding confirmed clamped."""

    def unclamped(self) -> bool:
        """Workholding confirmed released by its own sensor.

        Never inferred from `not clamped()`. On machines configured with
        clamp_released_sensor: false, the driver must always return False (never
        reads as released), and the controller uses the unclamp_fallback pull test.
        """

    def cycle_running(self) -> bool: ...

    def cycle_done(self) -> bool:
        """Latched true when a cycle ends normally; cleared by the next cycle_start()."""

    def feed_hold_active(self) -> bool: ...
    def alarm(self) -> bool: ...
