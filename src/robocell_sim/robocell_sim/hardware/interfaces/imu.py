"""IMU of a robot."""

from typing import Protocol


class Imu(Protocol):
    """IMU that measures how the robot turns and accelerates."""

    def rate(self) -> tuple[float, float, float]:
        """Return the angular velocity in rad/s about x, y and z."""
        ...

    def acceleration(self) -> tuple[float, float, float]:
        """Return the linear acceleration in m/s², gravity included."""
        ...
