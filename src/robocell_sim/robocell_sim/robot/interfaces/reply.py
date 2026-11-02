"""Validation of a pending SDO startup response."""

from typing import Protocol

from robocell_sim.robot.interfaces.transmission import Progress


class Reply(Protocol):
    """Validation of one expedited SDO response against its request."""

    def accept(self, payload: bytes, moment: float) -> Progress:
        """Complete a verified exchange or poll an unfinished transition."""
        ...

    def confirm(self, payload: bytes, moment: float) -> None:
        """Reject fault states and overdue drive transitions."""
        ...
