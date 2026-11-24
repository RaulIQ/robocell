"""ROS inputs to the shared body-command path."""

from typing import Protocol

from geometry_msgs.msg import Twist, TwistStamped  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.interfaces.pilot import Command


class Steering(Protocol):
    """Body commands received from ROS and the keyboard."""

    def listen(self) -> None:
        """Subscribe once to body commands and keyboard input."""
        ...

    def command(self, message: Twist) -> None:
        """Accept a new body command on its original arrival clock."""
        ...

    def keyboard(self, message: TwistStamped) -> None:
        """Accept keyboard intent without concealing its transport age."""
        ...

    def reading(self) -> Command:
        """Return the latest command with its source time."""
        ...
