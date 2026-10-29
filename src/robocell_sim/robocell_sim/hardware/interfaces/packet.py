"""Bytes of one device response."""

from typing import Protocol


class Packet(Protocol):
    """Packet that encodes one measurement in its device's wire format."""

    def encode(self) -> bytes:
        """Encode the complete packet."""
        ...
