"""Joint feedback acquired through CANopen PDOs."""

from dataclasses import dataclass
from typing import final, override

from rclpy.time import Time
from sensor_msgs.msg import JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.pilot import Joints
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Reported(Report):
    """Report using acquisition time and measured positions and velocities."""

    _sample: Joints
    _names: tuple[str, str]
    _topic: Topic

    @override
    def publish(self) -> None:
        self._topic.publish(
            JointState(
                header=Header(stamp=Time(nanoseconds=self._sample[0]).to_msg()),
                name=list(self._names),
                position=list(self._sample[1]),
                velocity=list(self._sample[2]),
            )
        )
