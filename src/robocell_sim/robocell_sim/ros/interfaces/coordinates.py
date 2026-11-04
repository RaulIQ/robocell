"""Coordinates of acquired points at a shared reference time."""

from typing import Protocol


class Coordinates(Protocol):
    """Projection from an acquisition pose into a reference pose."""

    def point(
        self, point: tuple[float, float], stamp: int, reference: int
    ) -> tuple[float, float]:
        """Express an acquired planar point at the reference time."""
        ...
