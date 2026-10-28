"""Instruments whose topics a ROS 2 node gears up."""

import math
import queue
from dataclasses import dataclass
from typing import final, override

from nav_msgs.msg import Odometry  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_msgs.msg import TFMessage  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.instruments import Instruments
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Geared(Instruments):
    """Instruments that open each topic as a publisher or subscriber of a node.

    The wheels name the left and the right joint in /joint_states, the
    resolution turns their angles into whole encoder counts, and the frame
    names where the odometry lies. Every call opens one more publisher or
    subscription, so the caller opens each topic once.
    """

    _node: Node
    _wheels: tuple[str, str]
    _resolution: int
    _frame: str

    @override
    def pose(self) -> Topic:
        return self._node.create_publisher(Odometry, "/odom", 10)

    @override
    def transforms(self) -> Topic:
        return self._node.create_publisher(TFMessage, "/tf", 10)

    @override
    def sense(self, inbox: queue.SimpleQueue[tuple[float, float]]) -> None:
        def receive(message: Imu) -> None:
            inbox.put(
                (
                    float(message.angular_velocity.z),
                    Time.from_msg(message.header.stamp).nanoseconds * 1e-9,
                )
            )

        self._node.create_subscription(
            Imu, "/imu", receive, qos_profile_sensor_data
        )

    @override
    def count(self, inbox: queue.SimpleQueue[tuple[int, int, Header]]) -> None:
        def receive(message: JointState) -> None:
            angles: dict[str, float] = dict(
                zip(message.name, message.position, strict=True)
            )
            inbox.put(
                (
                    round(
                        angles[self._wheels[0]]
                        * self._resolution
                        / (2 * math.pi)
                    ),
                    round(
                        angles[self._wheels[1]]
                        * self._resolution
                        / (2 * math.pi)
                    ),
                    Header(stamp=message.header.stamp, frame_id=self._frame),
                )
            )

        self._node.create_subscription(JointState, "/joint_states", receive, 10)
