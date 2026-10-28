#!/usr/bin/env python3
"""Odometry that reckons the pose of the robot from /joint_states and /imu."""

import contextlib
import queue

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.implementations.physics.joined import Joined
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.still import Still
from robocell_sim.robot.implementations.odometry.unread import Unread
from robocell_sim.robot.interfaces.odometry import Odometry
from robocell_sim.robot.interfaces.turn import Turn
from robocell_sim.ros.implementations.odometry.anchored import Anchored
from robocell_sim.ros.implementations.odometry.estimated import Estimated
from robocell_sim.ros.implementations.settings.declared import Declared
from robocell_sim.ros.implementations.wiring.geared import Geared

rclpy.init()
node = rclpy.create_node("odometry")
settings = Declared(node)
resolution = settings.whole("resolution", 4096)
wheels = (
    settings.text("left_joint", "wheel_left_joint"),
    settings.text("right_joint", "wheel_right_joint"),
)
frame = settings.text("odom_frame", "odom")
child = settings.text("base_frame", "base_footprint")
drive = Differential(
    settings.number("wheel_radius", 0.033),
    settings.number("wheel_separation", 0.288),
)
rates: queue.SimpleQueue[tuple[float, float]] = queue.SimpleQueue()
counts: queue.SimpleQueue[tuple[int, int, Header]] = queue.SimpleQueue()
instruments = Geared(node, wheels, resolution, frame)
instruments.sense(rates)
instruments.count(counts)
pose = instruments.pose()
transforms = instruments.transforms()
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
