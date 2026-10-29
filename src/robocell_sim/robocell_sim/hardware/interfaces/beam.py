"""A single lidar measurement at a requested heading."""

from typing import Protocol


class Beam(Protocol):
    """Beam that measures the world clockwise from the scanner's front."""

    def distance(self, angle: float) -> float:
        """Measure meters at clockwise degrees, inf without an echo."""
        ...
