"""Turn of a robot."""

from typing import Protocol

from robocell_sim.robot.interfaces.odometry import Odometry


class Turn(Protocol):
    """Turn of a robot that a gyro adds up from its yaw rates."""

    def read(self, rate: float, stamp: float) -> "Turn":
        """Return the turn after one more yaw rate in rad/s at the stamp."""
        ...

    def apply(self, odometry: Odometry, left: int, right: int) -> Odometry:
        """Return the odometry after the wheels reach these counts."""
        ...
