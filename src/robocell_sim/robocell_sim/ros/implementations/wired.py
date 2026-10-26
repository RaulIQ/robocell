"""Harness whose topics a ROS 2 node wires up."""

import queue
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Twist  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.harness import Harness
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Wired(Harness):
    """Harness that opens each topic as a publisher or subscription of one node.

    The node advertises and subscribes to the topics, so every call opens
    one more publisher or subscription and the caller opens each topic once.
    Subscriptions deliver messages only while the caller spins the node.
    """

    _node: Node

    @override
    def joints(self) -> Topic:
        return self._node.create_publisher(JointState, "/joint_states", 10)

    @override
    def scan(self) -> Topic:
        return self._node.create_publisher(LaserScan, "/scan", 10)

    @override
    def listen(self, inbox: queue.SimpleQueue[tuple[float, float]]) -> None:
        def receive(message: Twist) -> None:
            inbox.put((message.linear.x, message.angular.z))

        self._node.create_subscription(Twist, "/cmd_vel", receive, 10)
