"""Listening of a bounded CANopen host session."""

from typing import Protocol

import can

from robocell_sim.robot.interfaces.cycle import Cycle
from robocell_sim.robot.interfaces.pilot import Session


class Listening(Cycle, Protocol):
    """Bounded traffic routing and validation for both drives."""

    def read(self, message: can.Message, moment: float) -> Session:
        """Route one application frame to its owning drive."""
        ...

    def verify(self, message: can.Message, offset: int) -> None:
        """Reject emergency and unexpected restart during operation."""
        ...
