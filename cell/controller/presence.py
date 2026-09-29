"""Fixture seat sensor filtering. SAFETY-RELEVANT: changes here need human review.

Coolant, chips and vibration can make a seat sensor drop out briefly. Filtering must
never turn that into a false "safe" answer, so it works in the cautious direction
both ways:
  - seated(): while cutting, a dropout shorter than the window is ignored; a real
    loss of seat reads False once it lasts the window.
  - empty_confirmed(): before loading, "no part" must read continuously for the
    whole window, so one blink can't make an occupied fixture look empty.
The raw reading is still what CLAMP waits on and what cycle start requires: the
debounce never lets a cut *begin* on a part that isn't seated right now.
"""

from __future__ import annotations

from collections.abc import Callable

from cell.clock import Clock


class PartPresence:
    def __init__(self, read: Callable[[], bool], clock: Clock, window_s: float) -> None:
        self._read = read
        self._clock = clock
        self._window = window_s
        self._false_since: float | None = None

    def sample(self) -> None:
        """Call every controller tick, so a dropout streak is measured continuously and
        never inherited from a reading taken states ago."""
        self._sample()

    def _sample(self) -> tuple[bool, float]:
        now = self._clock.now()
        if self._read():
            self._false_since = None
            return True, now
        if self._false_since is None:
            self._false_since = now
        return False, now

    def seated(self) -> bool:
        present, now = self._sample()
        return present or (self._false_since is not None and now - self._false_since < self._window)

    def empty_confirmed(self) -> bool:
        present, now = self._sample()
        return (
            not present
            and self._false_since is not None
            and now - self._false_since >= self._window
        )
