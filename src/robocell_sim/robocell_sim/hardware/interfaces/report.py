"""Report of the simulation."""

from typing import Protocol


class Report(Protocol):
    """Report that tells others what the simulation looks like right now."""

    def publish(self) -> None:
        """Send the current state of the simulation as one message."""
        ...
