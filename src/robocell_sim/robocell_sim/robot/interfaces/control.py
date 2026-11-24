"""One aged body-command path for every operator source."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Command


class Control(Protocol):
    """Application input with its original source time."""

    def steer(self, velocity: tuple[float, float], moment: float) -> "Control":
        """Accept a newer source command with its original monotonic time."""
        ...

    def reading(self) -> Command:
        """Return body velocity with its original source time."""
        ...
