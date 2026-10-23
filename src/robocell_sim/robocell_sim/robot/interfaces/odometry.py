"""Odometry of a wheeled robot."""

from typing import Protocol


class Odometry(Protocol):
    """Pose of a robot reckoned from its wheel encoders and its gyro."""

    def read(self, left: int, right: int, angle: float) -> "Odometry":
        """Odometry after the encoders and the gyro reach these totals."""
        ...

    def position(self) -> tuple[float, float]:
        """Position of the robot on the floor in meters."""
        ...

    def heading(self) -> float:
        """Heading of the robot in radians, counterclockwise from the x axis."""
        ...
