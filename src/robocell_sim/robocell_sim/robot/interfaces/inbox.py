"""Inbox of the robot on the CAN bus."""

from typing import Protocol


class Inbox(Protocol):
    """Inbox that holds the frames a CAN bus delivered to the robot."""

    def frames(self) -> tuple[tuple[int, bytes], ...]:
        """Return every waiting frame, oldest first, without blocking."""
        ...
