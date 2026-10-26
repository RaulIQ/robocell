"""Harness of the robot in ROS 2."""

import queue
from typing import Protocol

from robocell_sim.ros.interfaces.topic import Topic


class Harness(Protocol):
    """Harness that opens the topics the real TurtleBot3 offers ROS 2.

    The driver of the real robot opens the same topics, so the rest of
    ROS 2 cannot tell the simulation from the hardware by them.
    """

    def joints(self) -> Topic:
        """Topic of the wheel angles, which robot_state_publisher reads."""
        ...

    def scan(self) -> Topic:
        """Topic of the ranges the lidar reads around the robot."""
        ...

    def picture(self) -> Topic:
        """Topic of what the camera sees, sent best effort like any video."""
        ...

    def calibration(self) -> Topic:
        """Topic of how the camera projects the world onto its pixels."""
        ...

    def listen(self, inbox: queue.SimpleQueue[tuple[float, float]]) -> None:
        """Queue every velocity command as forward speed and yaw rate."""
        ...
