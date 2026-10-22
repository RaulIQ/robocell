"""Route of the robot on the floor."""

from typing import Protocol


class Route(Protocol):
    """Route that compares where the encoders and MuJoCo put the robot."""

    def mark(
        self, estimate: tuple[float, float], truth: tuple[float, float]
    ) -> None:
        """Add the encoder and the true position in meters to the route."""

    def show(self) -> None:
        """Draw the route in a window until the window is closed."""
