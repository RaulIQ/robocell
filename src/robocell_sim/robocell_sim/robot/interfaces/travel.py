"""Travel of a wheeled robot."""

from typing import Protocol


class Travel(Protocol):
    """Pose and velocity of a robot reckoned from its wheel encoders alone.

    No gyro takes part, so a filter that fuses the gyro on its own counts
    each sensor once.
    """

    def read(self, left: int, right: int, moment: float) -> "Travel":
        """Travel after the encoders reach these totals at the moment in s."""
        ...

    def position(self) -> tuple[float, float]:
        """Position of the robot on the floor in meters."""
        ...

    def heading(self) -> float:
        """Heading of the robot in radians, counterclockwise from the x axis."""
        ...

    def velocity(self) -> tuple[float, float]:
        """Forward speed in m/s and yaw rate in rad/s over the last interval."""
        ...
