"""Stationary evidence for gyro calibration."""

from typing import Protocol


class Standstill(Protocol):
    """Evidence that wheels and acceleration agree with standing still."""

    def still(self) -> bool:
        """Whether fresh measurements support stationary calibration."""
        ...
