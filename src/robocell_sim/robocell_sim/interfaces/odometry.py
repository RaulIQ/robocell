"""Odometry of a wheeled robot."""

from typing import Protocol


class Odometry(Protocol):
    """Pose of a robot reckoned from the counts of its wheel encoders."""

    def read(self, left: int, right: int) -> "Odometry":
        """Odometry after the encoders reach these total counts."""

    def position(self) -> tuple[float, float]:
        """Position of the robot on the floor in meters."""
