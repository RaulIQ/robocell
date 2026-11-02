"""Pending protocol transmissions and validated startup responses."""

from typing import Protocol

from robocell_sim.robot.interfaces.negotiation import Exchange

Progress = tuple[tuple[Exchange, ...], tuple[float, float, int]]


class Transmission(Protocol):
    """A single pending exchange with bounded idempotent retransmission."""

    def advance(self, moment: float) -> Progress:
        """Send the next action or renew a timed-out request."""
        ...

    def renew(self, moment: float) -> Progress:
        """Retry the pending exchange within its request budget."""
        ...
