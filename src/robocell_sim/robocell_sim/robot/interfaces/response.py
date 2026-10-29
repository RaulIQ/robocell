"""The bytes of one bounded device response."""

from typing import Protocol


class Response(Protocol):
    """Response whose expected length is known from the device protocol."""

    def receive(self) -> bytes:
        """Read the complete response or fail within its deadline."""
        ...
