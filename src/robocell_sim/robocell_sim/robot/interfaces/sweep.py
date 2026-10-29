"""A complete turn of a planar laser scanner."""

from typing import Protocol


class Sweep(Protocol):
    """Sweep whose angular and temporal ordering describe the same rays."""

    def ranges(self) -> tuple[float, ...]:
        """Return meters on a uniform angular grid, inf for invalid rays."""
        ...

    def intensities(self) -> tuple[float, ...]:
        """Return device-specific return quality on the same grid."""
        ...

    def geometry(self) -> tuple[float, float, float]:
        """Return first angle, last angle and angular step in radians."""
        ...

    def timing(self) -> tuple[int, float, float]:
        """Return first-ray stamp, scan duration and time between rays."""
        ...
