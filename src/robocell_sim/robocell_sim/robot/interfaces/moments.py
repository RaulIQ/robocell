"""Online statistics for three-axis measurements."""

from typing import Protocol


class Moments(Protocol):
    """Sample count, mean and variance without retaining measurements."""

    def read(self, value: tuple[float, float, float]) -> "Moments":
        """Include one observation in the statistics."""
        ...

    def count(self) -> int:
        """Return the number of observations."""
        ...

    def mean(self) -> tuple[float, float, float]:
        """Return the mean on each axis."""
        ...

    def variance(self) -> tuple[float, float, float]:
        """Return unbiased sample variance, or fail before two samples."""
        ...
