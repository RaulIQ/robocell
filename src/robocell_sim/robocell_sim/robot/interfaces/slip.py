"""Slip of the wheels of a robot."""

from typing import Protocol


class Slip(Protocol):
    """Slip that shows when the encoders and the gyro disagree on a turn."""

    def read(
        self, left: int, right: int, angle: float, moment: float
    ) -> "Slip":
        """Slip after the encoders, gyro and clock reach these totals."""
        ...

    def slipping(self) -> bool:
        """Whether the wheels slipped between the last two readings."""
        ...
