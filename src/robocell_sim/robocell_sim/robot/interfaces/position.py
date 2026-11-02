"""Continuous wheel position from a bounded encoder count."""

from typing import Protocol


class Position(Protocol):
    """Wheel position that excludes ambiguous or unobserved travel."""

    def read(self, position: int, stamp: int) -> "Position":
        """Advance from a signed wire count acquired at a simulated time."""
        ...

    def reset(self) -> "Position":
        """Invalidate the baseline while retaining accumulated travel."""
        ...

    def angle(self) -> float:
        """Accumulated wheel position in radians."""
        ...
