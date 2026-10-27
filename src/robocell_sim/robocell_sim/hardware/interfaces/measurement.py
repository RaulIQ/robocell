"""Measurement of the IMU board."""

from typing import Protocol


class Measurement(Protocol):
    """Measurement of the IMU board as it goes out on the CAN bus."""

    def frames(self) -> tuple[tuple[int, bytes], ...]:
        """Return the identifier and payload of each frame in sending order."""
        ...
