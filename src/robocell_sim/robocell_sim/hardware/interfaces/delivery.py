"""The pending output of a paced serial link."""

from typing import Protocol


class Delivery(Protocol):
    """Delivery that retains bytes until the serial link accepts them."""

    def advance(self, moment: float) -> tuple[bytes, float]:
        """Transmit the due prefix and retain the remaining bytes and clock."""
        ...
