"""Atlas of maps from lidar scans."""

from typing import Protocol


class Atlas(Protocol):
    """Atlas that compares the maps by the true and by the reckoned pose."""

    def mark(
        self,
        truth: list[tuple[float, float]],
        estimate: list[tuple[float, float]],
    ) -> None:
        """Add the points of one scan placed by both poses to the maps."""

    def show(self) -> None:
        """Draw the maps in a window until the window is closed."""
