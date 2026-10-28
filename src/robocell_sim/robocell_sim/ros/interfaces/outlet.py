"""Outlet of the IMU driver in ROS 2."""

from typing import Protocol

from robocell_sim.ros.interfaces.topic import Topic


class Outlet(Protocol):
    """Outlet that opens the topic the IMU driver feeds ROS 2.

    The driver reads the board over CAN, so this topic is all of it that
    the rest of ROS 2 ever sees.
    """

    def imu(self) -> Topic:
        """Topic of the angular rates and the accelerations of the board."""
        ...
