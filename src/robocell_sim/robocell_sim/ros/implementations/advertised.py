"""Bridge whose topics a ROS 2 node advertises."""

import queue
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Twist  # type: ignore[import-untyped]  # ROS generates message modules without types
from nav_msgs.msg import Odometry  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.node import Node
from rosgraph_msgs.msg import Clock  # type: ignore[import-untyped]  # ROS generates message modules without types
from sensor_msgs.msg import JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_msgs.msg import TFMessage  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.bridge import Bridge
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Advertised(Bridge):
    """Bridge that opens each topic as a publisher or subscription of one node.

    The node advertises and subscribes to the topics, so every call opens
    one more publisher or subscription and the caller opens each topic once.
    Subscriptions deliver messages only while the caller spins the node.
    """

    _node: Node

    @override
    def clock(self) -> Topic:
        return self._node.create_publisher(Clock, "/clock", 10)

    @override
    def truth(self) -> Topic:
        return self._node.create_publisher(Odometry, "/ground_truth/odom", 10)

    @override
    def joints(self) -> Topic:
        return self._node.create_publisher(JointState, "/joint_states", 10)

    @override
    def frames(self) -> Topic:
        return self._node.create_publisher(TFMessage, "/tf", 10)

    @override
    def listen(self, inbox: queue.SimpleQueue[tuple[float, float]]) -> None:
        def receive(message: Twist) -> None:
            inbox.put((message.linear.x, message.angular.z))

        self._node.create_subscription(Twist, "/cmd_vel", receive, 10)
