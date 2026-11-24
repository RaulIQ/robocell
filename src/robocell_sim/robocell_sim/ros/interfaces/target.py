"""Transport of a simulated goal."""

from typing import Protocol

from geometry_msgs.msg import Twist  # type: ignore[import-untyped]  # ROS generates message modules without types


class Goal(Protocol):
    """A goal source driven by simulator time and ROS velocity commands."""

    def accept(self, message: Twist) -> None:
        """Receive a velocity command."""
        ...

    def advance(self, elapsed: float) -> None:
        """Advance the target by a simulation interval."""
        ...

    def publish(self, moment: float) -> None:
        """Send the current target at the given simulation time."""
        ...
