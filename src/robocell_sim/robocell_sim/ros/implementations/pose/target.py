"""The simulated cube as a Nav2 goal source."""

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Point, Pose, PoseStamped, Twist, Vector3  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from rclpy.publisher import Publisher
from rclpy.time import Time
from std_msgs.msg import ColorRGBA  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from tf2_geometry_msgs import do_transform_pose_stamped  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from tf2_ros import Buffer, TransformException  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from visualization_msgs.msg import Marker  # type: ignore[import-untyped]  # Jazzy ships this module without typing information

from robocell_sim.hardware.interfaces.prop import Prop
from robocell_sim.ros.interfaces.target import Goal

_TIMEOUT = 0.25


@final
@dataclass(frozen=True)
class Target(Goal):
    """Move the cube from fresh commands and publish its pose in the map."""

    _cube: Prop
    _origin: tuple[float, float, float]
    _output: tuple[Buffer, Publisher, Publisher]
    _intent: deque[tuple[float, float, float]]

    @override
    def accept(self, message: Twist) -> None:
        """Keep the latest bounded cube command with its arrival time."""
        linear, angular = message.linear.x, message.angular.z
        if math.isfinite(linear) and math.isfinite(angular):
            self._intent.append(
                (
                    max(-0.26, min(0.26, linear)),
                    max(-1.8, min(1.8, angular)),
                    time.monotonic(),
                )
            )

    @override
    def advance(self, elapsed: float) -> None:
        """Move on the simulation clock while commands remain fresh."""
        if self._intent:
            linear, angular, moment = self._intent[-1]
            if time.monotonic() - moment < _TIMEOUT:
                self._cube.move((linear, angular), elapsed)

    @override
    def publish(self, moment: float) -> None:
        """Combine the spawn transform with the latest SLAM correction."""
        buffer, topic, display = self._output
        try:
            transform = buffer.lookup_transform("map", "odom", Time())
        except TransformException:
            return
        goal = do_transform_pose_stamped(
            pose(self._cube.pose(), self._origin), transform
        )
        goal.header.stamp = Time(nanoseconds=round(moment * 1e9)).to_msg()
        topic.publish(goal)
        display.publish(marker(goal))


def pose(
    world: tuple[float, float, float], origin: tuple[float, float, float]
) -> PoseStamped:
    """Express the world pose relative to the robot spawn pose."""
    x, y, heading = world
    ox, oy, yaw = origin
    goal = PoseStamped()
    goal.header.frame_id = "odom"
    goal.pose.position.x = math.cos(yaw) * (x - ox) + math.sin(yaw) * (y - oy)
    goal.pose.position.y = -math.sin(yaw) * (x - ox) + math.cos(yaw) * (y - oy)
    goal.pose.orientation.z = math.sin((heading - yaw) / 2)
    goal.pose.orientation.w = math.cos((heading - yaw) / 2)
    return goal


def marker(goal: PoseStamped) -> Marker:
    """Show the cube on the map without changing the navigation goal."""
    return Marker(
        header=goal.header,
        ns="cube",
        type=Marker.CUBE,
        pose=Pose(
            position=Point(
                x=goal.pose.position.x, y=goal.pose.position.y, z=0.10
            ),
            orientation=goal.pose.orientation,
        ),
        scale=Vector3(x=0.20, y=0.20, z=0.20),
        color=ColorRGBA(r=1.0, g=0.35, b=0.05, a=1.0),
    )
