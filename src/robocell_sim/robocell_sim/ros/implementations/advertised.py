"""Bridge whose topics a ROS 2 node advertises."""

from dataclasses import dataclass
from typing import final, override

from nav_msgs.msg import Odometry  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.node import Node
from rosgraph_msgs.msg import Clock  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_msgs.msg import TFMessage  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.bridge import Bridge
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Advertised(Bridge):
    """Bridge that opens each topic as a publisher of one node.

    The node advertises the topics, so every call opens one more publisher
    and the caller opens each topic once.
    """

    _node: Node

    @override
    def clock(self) -> Topic:
        return self._node.create_publisher(Clock, "/clock", 10)

    @override
    def truth(self) -> Topic:
        return self._node.create_publisher(Odometry, "/ground_truth/odom", 10)

    @override
    def frames(self) -> Topic:
        return self._node.create_publisher(TFMessage, "/tf", 10)
