"""Journal of the IMU driver."""

from typing import Protocol


class Journal(Protocol):
    """Journal that hears how many measurements of the IMU went missing."""

    def lose(self, count: int) -> None:
        """Take note that the last frame revealed count lost measurements."""
        ...
