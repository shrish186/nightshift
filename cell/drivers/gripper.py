"""Gripper interface. Commands are non-blocking; poll is_closed()/has_part()."""

from __future__ import annotations

from typing import Protocol


class Gripper(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...

    def is_open(self) -> bool:
        """Jaws confirmed fully open."""

    def is_closed(self) -> bool:
        """Jaws finished closing (on a part or on nothing)."""

    def has_part(self) -> bool:
        """Part-present sensor. Only meaningful when is_closed()."""
