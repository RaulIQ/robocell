"""Instruments of the odometry in ROS 2."""

import queue
from typing import Protocol

from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.topic import Topic


class Instruments(Protocol):
    """Instruments that the odometry node reads and writes through ROS 2.

    The node hears the gyro and the wheel joints, and tells the rest of
    ROS 2 where the robot is as an odometry message and as a transform.
    Subscriptions deliver readings only while the caller spins the node.
    """

    def pose(self) -> Topic:
        """Topic of the pose the odometry reckons, as nav_msgs/Odometry."""
        ...

    def transforms(self) -> Topic:
        """Topic of the transform from odom to the body of the robot."""
        ...

    def sense(self, inbox: queue.SimpleQueue[tuple[float, float]]) -> None:
        """Queue every yaw rate of the gyro with its stamp in seconds."""
        ...

    def count(self, inbox: queue.SimpleQueue[tuple[int, int, Header]]) -> None:
        """Queue the left and right encoder counts with their header."""
        ...
