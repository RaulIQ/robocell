"""Cloud of lidar points on the floor."""

from typing import Protocol


class Cloud(Protocol):
    """Cloud of points where lidar rays echoed, in room axes."""

    def points(self) -> list[tuple[float, float]]:
        """Points on the floor in meters, in scan order."""
