"""System-time command timestamps mapped to monotonic arrival age."""

from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import TwistStamped  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.interfaces.pilot import Command
from robocell_sim.ros.interfaces.arrival import Arrival


@final
@dataclass(frozen=True)
class Stamped(Arrival):
    """Keyboard input whose source timestamp survives ROS transport delay."""

    _message: TwistStamped
    _time: tuple[float, int]

    @override
    def command(self) -> Command:
        age = (
            self._time[1]
            - self._message.header.stamp.sec * 1000000000
            - self._message.header.stamp.nanosec
        ) * 1e-9
        if age < 0:
            error = "Keyboard command has a future system-time timestamp"
            raise ValueError(error)
        return (
            self._message.twist.linear.x,
            self._message.twist.angular.z,
            self._time[0] - age,
        )
