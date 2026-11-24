"""Pose and motion of a simulated target."""

from typing import Protocol


class Prop(Protocol):
    """A kinematic target expressed in world coordinates."""

    def place(self) -> None:
        """Put the marker in the workshop."""
        ...

    def move(self, velocity: tuple[float, float], elapsed: float) -> None:
        """Apply a planar command for a simulation interval."""
        ...

    def pose(self) -> tuple[float, float, float]:
        """Return world x, y and heading."""
        ...
