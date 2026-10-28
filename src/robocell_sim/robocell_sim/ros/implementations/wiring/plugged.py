"""Outlet whose topic a ROS 2 node plugs in."""

from dataclasses import dataclass
from typing import final, override

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.outlet import Outlet
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Plugged(Outlet):
    """Outlet that opens its topic as a publisher of one node.

    The node advertises the topic best effort like any sensor stream, so
    every call opens one more publisher and the caller opens it once.
    """

    _node: Node

    @override
    def imu(self) -> Topic:
        return self._node.create_publisher(Imu, "/imu", qos_profile_sensor_data)
