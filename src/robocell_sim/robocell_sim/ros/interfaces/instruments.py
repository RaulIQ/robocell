"""Instruments of the odometry in ROS 2."""

import queue
from typing import Protocol

from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.topic import Topic


class Instruments(Protocol):
    """Instruments that the odometry node reads and writes through ROS 2.

    The node hears the wheel joints alone and tells the rest of ROS 2 what
    the wheels reckon, as an odometry message and as a transform. The gyro
    stays out, so a filter that fuses it counts it once. Subscriptions
    deliver readings only while the caller spins the node.
    """

    def pose(self) -> Topic:
        """Topic of the wheel odometry, as nav_msgs/Odometry."""
        ...

    def transforms(self) -> Topic:
        """Topic of the transform from odom to the body of the robot."""
        ...

    def count(
        self, inbox: queue.SimpleQueue[tuple[int, int, float, Header]]
    ) -> None:
        """Queue the left and right counts with their moment and header."""
        ...
