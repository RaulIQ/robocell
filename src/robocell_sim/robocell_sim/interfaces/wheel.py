"""Wheel of a robot."""

from typing import Protocol


class Wheel(Protocol):
    """Wheel whose encoder counts how far it has turned."""

    def ticks(self) -> int:
        """Whole ticks turned since the start, negative when turned back."""
