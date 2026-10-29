"""A nonblocking byte stream."""

from typing import Protocol


class Port(Protocol):
    """Port that exchanges bytes without waiting for the other endpoint."""

    def receive(self) -> bytes:
        """Read available bytes, or an empty chunk when none have arrived."""
        ...

    def send(self, payload: bytes) -> int:
        """Write available capacity and report the accepted byte count."""
        ...
