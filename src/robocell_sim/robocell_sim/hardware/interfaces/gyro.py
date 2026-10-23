"""Gyro of a robot."""

from typing import Protocol


class Gyro(Protocol):
    """Gyro whose turn rate adds up to the angle the robot has turned."""

    def angle(self) -> float:
        """Radians turned counterclockwise since the start."""
        ...
