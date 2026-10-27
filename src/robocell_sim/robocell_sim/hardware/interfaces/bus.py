"""Bus of the robot."""

from typing import Protocol


class Bus(Protocol):
    """Bus that carries frames from a board to every device that listens."""

    def send(self, identifier: int, payload: bytes) -> None:
        """Put one frame with an 11-bit identifier on the bus."""
        ...
