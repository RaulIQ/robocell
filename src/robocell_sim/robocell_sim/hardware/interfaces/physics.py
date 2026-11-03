"""Physics of the simulation."""

from typing import Protocol


class Physics(Protocol):
    """Physics that move the simulation forward in time."""

    def advance(self, moment: float) -> None:
        """Advance physics toward the requested time in a bounded batch."""
        ...
