"""ROS inputs to the shared body-command path."""

from typing import Protocol

from geometry_msgs.msg import Twist, TwistStamped  # type: ignore[import-untyped]  # ROS generates message modules without types
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.interfaces.pilot import Command


class Steering(Protocol):
    """Source commands and guard observations kept on independent clocks."""

    def listen(self) -> None:
        """Subscribe once to body commands, keyboard input and lidar scans."""
        ...

    def command(self, message: Twist) -> None:
        """Accept a new body command on its original arrival clock."""
        ...

    def keyboard(self, message: TwistStamped) -> None:
        """Accept keyboard intent without concealing its transport age."""
        ...

    def scan(self, message: LaserScan) -> None:
        """Replace collision observations without renewing source commands."""
        ...

    def reading(self, clock: int) -> Command:
        """Evaluate the current guard with the original command deadline."""
        ...
