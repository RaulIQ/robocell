"""Lidar of a robot."""

from typing import Protocol


class Lidar(Protocol):
    """Lidar that ranges the world around the robot in a level plane."""

    def ranges(self) -> tuple[float, ...]:
        """Meters per degree counterclockwise from ahead, inf with no echo."""
