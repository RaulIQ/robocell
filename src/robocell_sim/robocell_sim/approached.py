#!/usr/bin/env python3
"""Convert observed target positions into Nav2 approach goals in odometry."""

import math
from collections import deque
from collections.abc import Callable

import rclpy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.utilities import try_shutdown

from robocell_sim.robot.implementations.detection.recent import Recent
from robocell_sim.robot.implementations.following.approached import Approached
from robocell_sim.ros.implementations.clock.timed import Timed
from robocell_sim.ros.implementations.pose.goal import Goal
from robocell_sim.ros.implementations.pose.offered import Offered
from robocell_sim.ros.messages import (
    Buffer,
    PoseStamped,
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


goals = Offered(
    observations, buffer, Goal(Approached(distance)), (output, recent, warn)
)
node.create_subscription(PoseStamped, "/target/pose", remember, 1)
node.create_timer(0.1, goals.publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    destroy()
    try_shutdown()
