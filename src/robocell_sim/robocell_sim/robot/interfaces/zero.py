"""Stationary gyro calibration."""

from typing import Protocol


class Zero(Protocol):
    """Estimate of a constant gyro offset from a stationary interval."""

    def read(
        self, rate: tuple[float, float, float], moment: float, *, still: bool
    ) -> "Zero":
        """Observe one measurement without adapting an established offset."""
        ...

    def ready(self) -> bool:
        """Whether a complete stationary window establishes the offset."""
        ...

    def bias(self) -> tuple[float, float, float]:
        """Return the established offset, or fail before calibration."""
        ...

    def variance(self) -> tuple[float, float, float]:
        """Return the uncertainty of the estimated mean on each axis."""
        ...
