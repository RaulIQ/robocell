#!/usr/bin/env python3
"""Deskew raw driver scans onto a fixed grid dedicated to SLAM."""

import contextlib
import logging
import math
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_ros import Buffer, TransformException, TransformListener  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.implementations.lidar.rectified import Rectified
from robocell_sim.ros.implementations.lidar.transformed import Transformed
from robocell_sim.ros.implementations.settings.declared import Declared

rclpy.init()
node = rclpy.create_node("slam_scan")
settings = Declared(node)
source = settings.text("input", "/scan")
destination = settings.text("output", "/scan/slam")
fixed = settings.text("fixed_frame", "odom")
count = settings.whole("bins", 360)
timeout = settings.number("transform_timeout", 0.5)
capacity = settings.whole("capacity", 8)
if (
    source == destination
    or count <= 1
    or capacity < 1
    or not math.isfinite(timeout)
    or timeout <= 0
):
    message = "SLAM adapter requires distinct topics and positive limits"
    raise ValueError(message)
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
topic = node.create_publisher(LaserScan, destination, 10)
pending: deque[tuple[LaserScan, float]] = deque()


def receive(message: LaserScan) -> None:
    """Keep a bounded queue without altering the driver's message."""
    if len(pending) >= capacity:
        pending.popleft()
        logging.getLogger("slam_scan").warning(
            "SLAM scan queue overflow dropped its oldest rotation"
        )
    pending.append((message, time.monotonic() + timeout))


node.create_subscription(LaserScan, source, receive, qos_profile_sensor_data)
with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
    while node.context.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        while pending:
            scan, deadline = pending[0]
            if time.monotonic() >= deadline:
                pending.popleft()
                logging.getLogger("slam_scan").warning(
                    "SLAM scan expired while waiting for odometry TF"
                )
                continue
            try:
                Rectified(
                    scan,
                    Transformed(buffer, scan.header.frame_id, fixed),
                    count,
                    topic,
                ).publish()
            except TransformException:
                break
            except ValueError as failure:
                logging.getLogger("slam_scan").warning(
                    "SLAM scan rejected because %s", failure
                )
            pending.popleft()
node.context.try_shutdown()
