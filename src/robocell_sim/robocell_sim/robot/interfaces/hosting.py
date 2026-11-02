"""Lifetime of a standalone CANopen host."""

from contextlib import AbstractContextManager
from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Pilot


class Hosting(Protocol):
    """One bounded connection attempt with explicit recovery permission."""

    def connection(
        self, moment: float, *, recovery: bool
    ) -> AbstractContextManager[Pilot]:
        """Open a validated transport and inhibit both drives on exit."""
        ...
