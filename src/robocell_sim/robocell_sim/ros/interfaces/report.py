"""Report to ROS 2."""

from typing import Protocol


class Report(Protocol):
    """Report that tells ROS 2 what the simulation looks like right now."""

    def publish(self) -> None:
        """Send the current state of the simulation as one message."""
        ...
