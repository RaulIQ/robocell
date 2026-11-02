"""Keyboard body commands carry the actual source event's wall time."""

from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Twist, TwistStamped, Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.time import Time
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.command import Command
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Keyed(Report):
    """Keyboard intent stamped at the key event, rather than guard output."""

    _command: Command
    _stamp: int
    _topic: Topic

    @override
    def publish(self) -> None:
        linear, angular = self._command.velocity()
        self._topic.publish(
            TwistStamped(
                header=Header(
                    stamp=Time(nanoseconds=self._stamp).to_msg(),
                    frame_id="base_link",
                ),
                twist=Twist(
                    linear=Vector3(x=linear), angular=Vector3(z=angular)
                ),
            )
        )
