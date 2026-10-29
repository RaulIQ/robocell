"""A command with its expected response."""

from typing import Protocol


class Transaction(Protocol):
    """Transaction that validates descriptor framing before returning data."""

    def answer(self, command: bytes, descriptor: bytes, size: int) -> bytes:
        """Send the command and return its validated response payload."""
        ...
