"""Physics of the simulation."""

from typing import Protocol


class Physics(Protocol):
    """Physics that move the simulation forward in time."""

    def advance(self, moment: float) -> None:
        """Step the physics until its clock reaches the moment in seconds."""
