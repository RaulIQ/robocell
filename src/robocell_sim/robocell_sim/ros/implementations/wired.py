"""Harness whose topics a ROS 2 node wires up."""

import queue
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Twist  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types

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
    def picture(self) -> Topic:
        return self._node.create_publisher(
            Image, "/camera/image_raw", qos_profile_sensor_data
        )

    @override
    def calibration(self) -> Topic:
        return self._node.create_publisher(
            CameraInfo, "/camera/camera_info", 10
        )

    @override
    def listen(self, inbox: queue.SimpleQueue[tuple[float, float]]) -> None:
        def receive(message: Twist) -> None:
            inbox.put((message.linear.x, message.angular.z))

        self._node.create_subscription(Twist, "/cmd_vel", receive, 10)
