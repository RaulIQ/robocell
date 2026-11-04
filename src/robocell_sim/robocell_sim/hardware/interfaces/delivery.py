"""The pending output of a paced serial link."""

from typing import Protocol


class Delivery(Protocol):
    """Delivery that retains bytes until the serial link accepts them."""

    def advance(self, moment: float, backlog: int) -> tuple[bytes, float]:
        """Transmit bytes with the wire time owed to the previous backlog."""
        ...
