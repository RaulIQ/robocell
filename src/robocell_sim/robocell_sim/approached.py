#!/usr/bin/env python3
"""Convert observed target positions into Nav2 approach goals in odometry."""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robocell_sim.approach import approach
from robocell_sim.freshness import fresh

rclpy.init()
node = rclpy.create_node("approached")
distance = float(node.declare_parameter("distance", 1.0).value)
if not math.isfinite(distance) or distance <= 0:
    raise ValueError("Following distance must be finite and positive")
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
observations: deque[PoseStamped] = deque(maxlen=1)
output = node.create_publisher(PoseStamped, "/goal_update", 1)
node.create_subscription(
    PoseStamped,
    "/target/pose",
    lambda observation: observations.append(observation),
    1,
)


def publish() -> None:
    """Use fresh observations and the estimated robot pose to choose a goal."""
    if not observations:
        return
    observation = observations[-1]
    if observation.header.frame_id != "odom":
        node.get_logger().error("Target observation requires the odom frame")
        return
    if not fresh(
        Time.from_msg(observation.header.stamp).nanoseconds / 1e9,
        node.get_clock().now().nanoseconds / 1e9,
    ):
        return
    try:
        transform = buffer.lookup_transform("odom", "base_footprint", Time())
    except TransformException:
        return
    goal = PoseStamped()
    goal.header = observation.header
    goal.pose.position.x, goal.pose.position.y = approach(
        (observation.pose.position.x, observation.pose.position.y),
        (transform.transform.translation.x, transform.transform.translation.y),
        distance,
    )
    angle = math.atan2(
        observation.pose.position.y - transform.transform.translation.y,
        observation.pose.position.x - transform.transform.translation.x,
    )
    goal.pose.orientation.z = math.sin(angle / 2)
    goal.pose.orientation.w = math.cos(angle / 2)
    output.publish(goal)


node.create_timer(0.1, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
