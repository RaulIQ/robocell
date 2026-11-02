"""A drive session advanced at one monotonic instant."""

from typing import Protocol

from robocell_sim.hardware.interfaces.session import Session


class Progression(Protocol):
    """Progression that returns the session after servicing one concern."""

    def advance(self, moment: float) -> Session:
        """Service the session at monotonic wall time."""
        ...
