#!/usr/bin/env python3
"""Convert observed target positions into Nav2 approach goals in odometry."""

import math
from collections import deque
from collections.abc import Callable

import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.time import Time
from rclpy.utilities import try_shutdown

from robocell_sim.approach import approach
from robocell_sim.robot.implementations.detection.recent import Recent
from robocell_sim.ros.implementations.clock.timed import Timed
from robocell_sim.ros.messages import (
    Buffer,
    PoseStamped,
    TransformException,
    TransformListener,
)

rclpy.init()
node = rclpy.create_node("approached")
recent = Recent(Timed(node))
node_logger: Callable[[], RcutilsLogger] = node.get_logger
warn: Callable[[str], None] = node_logger().error
destroy: Callable[[], None] = node.destroy_node
distance = float(node.declare_parameter("distance", 1.0).value)
if not math.isfinite(distance) or distance <= 0:
    error = "Following distance must be finite and positive"
    raise ValueError(error)
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
observations: deque[PoseStamped] = deque(maxlen=1)
output = node.create_publisher(PoseStamped, "/goal_update", 1)


def remember(observation: PoseStamped) -> None:
    """Keep the latest observation received from the camera."""
    observations.append(observation)


def usable(observation: PoseStamped) -> bool:
    """Check the observation frame and acquisition time."""
    if observation.header.frame_id != "odom":
        warn("Target observation requires the odom frame")
        return False
    return recent(Time.from_msg(observation.header.stamp).nanoseconds)


def destination(
    observation: PoseStamped, robot: tuple[float, float]
) -> PoseStamped:
    """Choose a point at the following distance and face the observed target."""
    position = observation.pose.position
    goal = PoseStamped()
    goal.header = observation.header
    goal.pose.position.x, goal.pose.position.y = approach(
        (position.x, position.y), robot, distance
    )
    angle = math.atan2(position.y - robot[1], position.x - robot[0])
    goal.pose.orientation.z = math.sin(angle / 2)
    goal.pose.orientation.w = math.cos(angle / 2)
    return goal


def publish() -> None:
    """Publish a goal when a fresh target and robot transform are available."""
    if not observations or not usable(observations[-1]):
        return
    try:
        transform = buffer.lookup_transform("odom", "base_footprint", Time())
    except TransformException:
        pass
    else:
        robot = transform.transform.translation
        output.publish(destination(observations[-1], (robot.x, robot.y)))


node.create_subscription(PoseStamped, "/target/pose", remember, 1)
node.create_timer(0.1, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    destroy()
    try_shutdown()
