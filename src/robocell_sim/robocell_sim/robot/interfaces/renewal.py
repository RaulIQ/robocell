"""Renewal of a bounded CANopen host session."""

from typing import Protocol

from robocell_sim.robot.interfaces.cycle import Cycle


class Renewal(Cycle, Protocol):
    """Cyclic transmission retaining the age of application input."""

    def transmit(self, moment: float) -> None:
        """Transmit fresh targets only to verified enabled drives."""
        ...
