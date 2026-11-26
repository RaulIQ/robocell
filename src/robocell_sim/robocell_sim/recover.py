#!/usr/bin/env python3
"""Switch RTAB-Map between mapping and recovery within one session."""

import math
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from robocell_sim.recovery import blocked, confirmed

rclpy.init()
try:
    node = Node("mapping_recovery")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    clients = {
        mode: node.create_client(Empty, f"/rtabmap/set_mode_{mode}")
        for mode in ("mapping", "localization")
    }
    mode = "mapping"
    requested = mode
    pending = None
    sent = 0.0
    odometry = Odometry()
    anchor = None
    loaded = False
    confirmations = deque(maxlen=2)
    last_match = 0

    def receive(message: Odometry) -> None:
        global odometry
        odometry = message

    def observe(message: LaserScan) -> None:
        global anchor, requested
        if not loaded or mode != "mapping" or requested != mode:
            return
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        age = abs(stamp - Time.from_msg(odometry.header.stamp).nanoseconds)
        if age > 200_000_000 or abs(odometry.twist.twist.angular.z) > 0.18:
            anchor = None
            return
        point = odometry.pose.pose.position
        if anchor is None:
            anchor = (stamp, point.x, point.y, message.ranges)
            return
        elapsed = stamp - anchor[0]
        if elapsed < 0 or elapsed > 1_500_000_000:
            anchor = (stamp, point.x, point.y, message.ranges)
            return
        if elapsed >= 1_000_000_000:
            travel = math.hypot(point.x - anchor[1], point.y - anchor[2])
            if blocked(anchor[3], message.ranges, travel):
                requested = "localization"
            anchor = (stamp, point.x, point.y, message.ranges)

    def match(message: Info) -> None:
        global loaded, requested, last_match
        loaded = loaded or len(message.wm_state) >= 3
        if mode != "localization" or requested != mode:
            return
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        if stamp <= last_match:
            return
        if not (message.loop_closure_id > 0 or message.proximity_detection_id > 0):
            confirmations.clear()
            return
        try:
            transform = buffer.lookup_transform("map", "odom", Time()).transform
        except TransformException:
            return
        last_match = stamp
        rotation = transform.rotation
        heading = math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y**2 + rotation.z**2),
        )
        correction = (transform.translation.x, transform.translation.y, heading)
        if confirmed(correction, confirmations):
            requested = "mapping"
        confirmations.append(correction)

    def switch() -> None:
        global pending, sent, mode, anchor
        if pending is not None:
            if pending.done():
                response = pending.result()
                pending = None
                if response is not None:
                    mode = requested
                    anchor = None
                    confirmations.clear()
                    node.get_logger().info(f"RTAB-Map switched to {mode}")
            elif time.monotonic() - sent > 5:
                clients[requested].remove_pending_request(pending)
                pending = None
                node.get_logger().warning(f"RTAB-Map did not confirm {requested} mode")
            return
        if requested != mode and clients[requested].service_is_ready():
            pending = clients[requested].call_async(Empty.Request())
            sent = time.monotonic()

    node.create_subscription(
        Odometry, "/odometry/filtered", receive, qos_profile_sensor_data
    )
    node.create_subscription(LaserScan, "/scan/slam", observe, qos_profile_sensor_data)
    node.create_subscription(Info, "/info", match, qos_profile_sensor_data)
    node.create_timer(0.05, switch)
    rclpy.spin(node)
finally:
    rclpy.try_shutdown()
