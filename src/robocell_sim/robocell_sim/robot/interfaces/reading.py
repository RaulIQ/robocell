"""Reading of the IMU."""

from typing import Protocol


class Reading(Protocol):
    """Reading of the IMU in SI units, in the imu_link frame."""

    def rate(self) -> tuple[float, float, float]:
        """Return the angular velocity in rad/s about x, y and z."""
        ...

    def acceleration(self) -> tuple[float, float, float]:
        """Return the linear acceleration in m/s², gravity included."""
        ...
