#!/usr/bin/env python3
"""Odometry that reckons the pose of the robot from /joint_states and /imu."""

import contextlib
import math
import queue

import rclpy
from nav_msgs.msg import Odometry as Message  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_msgs.msg import TFMessage  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.implementations.physics.joined import Joined
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.still import Still
from robocell_sim.robot.implementations.odometry.unread import Unread
from robocell_sim.robot.interfaces.odometry import Odometry
from robocell_sim.robot.interfaces.turn import Turn
from robocell_sim.ros.implementations.odometry.anchored import Anchored
from robocell_sim.ros.implementations.odometry.estimated import Estimated

rclpy.init()
node = rclpy.create_node("odometry")
resolution = int(node.declare_parameter("resolution", 4096).value)
wheels = (
    str(node.declare_parameter("left_joint", "wheel_left_joint").value),
    str(node.declare_parameter("right_joint", "wheel_right_joint").value),
)
frame = str(node.declare_parameter("odom_frame", "odom").value)
child = str(node.declare_parameter("base_frame", "base_footprint").value)
drive = Differential(
    float(node.declare_parameter("wheel_radius", 0.033).value),
    float(node.declare_parameter("wheel_separation", 0.288).value),
)
rates: queue.SimpleQueue[tuple[float, float]] = queue.SimpleQueue()
counts: queue.SimpleQueue[tuple[int, int, Header]] = queue.SimpleQueue()


def rate(message: Imu) -> None:
    """Queue the yaw rate of one IMU reading with its stamp in seconds."""
    rates.put(
        (
            float(message.angular_velocity.z),
            Time.from_msg(message.header.stamp).nanoseconds * 1e-9,
        )
    )


def count(message: JointState) -> None:
    """Queue the wheel angles as whole encoder counts with their header."""
    angles = dict(zip(message.name, message.position, strict=True))
    counts.put(
        (
            round(angles[wheels[0]] * resolution / (2 * math.pi)),
            round(angles[wheels[1]] * resolution / (2 * math.pi)),
            Header(stamp=message.header.stamp, frame_id=frame),
        )
    )


node.create_subscription(Imu, "/imu", rate, qos_profile_sensor_data)
node.create_subscription(JointState, "/joint_states", count, 10)
pose = node.create_publisher(Message, "/odom", 10)
transforms = node.create_publisher(TFMessage, "/tf", 10)
turn: Turn = Still(0.0)
odometry: Odometry = Unread(drive, resolution, (0.0, 0.0, 0.0))
with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
    while node.context.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        while not rates.empty():
            turn = turn.read(*rates.get())
        while not counts.empty():
            left, right, header = counts.get()
            odometry = turn.apply(odometry, left, right)
            Joined(
                (
                    Estimated(odometry, header, child, pose),
                    Anchored(odometry, header, child, transforms),
                )
            ).publish()
node.context.try_shutdown()
