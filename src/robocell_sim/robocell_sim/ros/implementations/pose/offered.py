"""A fresh target pose offered to the navigation planner."""

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import final, override

from rclpy.time import Time

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.ros.implementations.pose.goal import Goal
from robocell_sim.ros.interfaces.topic import Topic
from robocell_sim.ros.messages import Buffer, PoseStamped, TransformException


@final
@dataclass(frozen=True)
class Offered(Report):
    """Publish a goal from the latest observation and current robot pose."""

    _observations: deque[PoseStamped]
    _buffer: Buffer
    _goal: Goal
    _output: tuple[Topic, Callable[[int], bool], Callable[[str], None]]

    @override
    def publish(self) -> None:
        topic, recent, warn = self._output
        if not self._observations:
            return
        observation = self._observations[-1]
        if observation.header.frame_id != "odom":
            warn("Target observation requires the odom frame")
        elif recent(Time.from_msg(observation.header.stamp).nanoseconds):
            try:
                robot = self._buffer.lookup_transform(
                    "odom", "base_footprint", Time()
                ).transform.translation
            except TransformException:
                return
            topic.publish(self._goal(observation, (robot.x, robot.y)))
