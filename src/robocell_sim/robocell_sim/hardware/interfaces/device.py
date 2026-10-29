"""A serial device at one point in its lifetime."""

from __future__ import annotations

from typing import Protocol


class Device(Protocol):
    """Device that advances its immutable session and returns wire bytes."""

    def exchange(self, payload: bytes, moment: float) -> tuple[Device, bytes]:
        """Accept host bytes and acquire measurements due at simulated time."""
        ...
