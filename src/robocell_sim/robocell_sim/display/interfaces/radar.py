"""Radar view of the lidar scans."""

from typing import Protocol


class Radar(Protocol):
    """Radar that shows the latest lidar scan around the robot."""

    def mark(self, scan: tuple[float, ...]) -> None:
        """Send the ranges of a scan in meters to the radar."""
        ...

    def show(self) -> None:
        """Draw the scans in a window until the window is closed."""
        ...
