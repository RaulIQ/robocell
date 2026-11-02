"""Service of a bounded CANopen host session."""

from typing import Protocol

from robocell_sim.robot.interfaces.cycle import Cycle
from robocell_sim.robot.interfaces.pilot import Session


class Service(Cycle, Protocol):
    """Startup and protection around the cyclic traffic."""

    def start(self, moment: float, clock: int) -> Session:
        """Advance both zero-motion startup sequences independently."""
        ...
