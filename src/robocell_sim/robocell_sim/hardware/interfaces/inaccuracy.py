"""Inaccuracy of a three-axis sensor."""

from typing import Protocol


class Inaccuracy(Protocol):
    """Inaccuracy that a sensor adds to what it measures on its three axes."""

    def reading(
        self, value: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        """Return what the sensor reads when x, y and z truly hold the value."""
        ...
