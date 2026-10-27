"""Frame of the IMU board."""

from typing import Protocol


class Frame(Protocol):
    """Frame of the IMU board as it came off the CAN bus."""

    def gyro(self) -> bool:
        """Tell whether the frame carries the gyro, not the accelerometer."""
        ...

    def number(self) -> int:
        """Return the number of the measurement the frame belongs to."""
        ...

    def values(self) -> tuple[int, int, int]:
        """Return x, y and z as raw counts of thousandths."""
        ...

    def gap(self, last: int) -> int:
        """Return how many measurement numbers lie between last and this."""
        ...
