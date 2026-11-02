"""One bounded host service cycle."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Session


class Cycle(Protocol):
    """An immutable session advanced by CAN input and protective deadlines."""

    def advance(self, moment: float, clock: int) -> Session:
        """Service the pair without blocking the next protective cycle."""
        ...
