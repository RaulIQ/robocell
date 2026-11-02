"""An application command with its original monotonic age."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Command


class Arrival(Protocol):
    """Transport timestamps mapped once into the host's monotonic clock."""

    def command(self) -> Command:
        """Return body velocity and the original monotonic reception time."""
        ...
