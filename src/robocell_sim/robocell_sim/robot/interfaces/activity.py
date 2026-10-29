"""Serial activity measured against device and wall clocks."""

from typing import Protocol


class Activity(Protocol):
    """Activity that distinguishes missing data from a paused device clock."""

    def read(
        self, payload: bytes, stamp: int, moment: float
    ) -> tuple[float, int, float]:
        """Update last data, last device time and last clock progress."""
        ...
