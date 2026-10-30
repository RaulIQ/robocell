"""A comparison of timestamped planar odometry streams."""

from typing import Protocol


class Comparison(Protocol):
    """Comparison of poses in world and velocities in the robot frame."""

    def rows(self) -> tuple[tuple[float, ...], ...]:
        """Return time, position error, heading error and two rate residuals."""
        ...

    def reference(self, moment: float) -> tuple[float, ...]:
        """Interpolate time, x, y, yaw, forward speed and yaw rate."""
        ...

    def errors(self, estimate: tuple[float, ...]) -> tuple[float, ...]:
        """Compare one odom estimate with the reference at its timestamp."""
        ...
