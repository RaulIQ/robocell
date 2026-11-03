"""Publishing IMU measurements after gyro offset compensation."""

from copy import deepcopy
from dataclasses import dataclass
from typing import final, override

from sensor_msgs.msg import Imu  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.zero import Zero
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Corrected(Report):
    """IMU report retaining acquisition metadata and offset uncertainty."""

    _message: Imu
    _zero: Zero
    _topic: Topic

    @override
    def publish(self) -> None:
        message = deepcopy(self._message)
        message.angular_velocity.x -= self._zero.bias()[0]
        message.angular_velocity.y -= self._zero.bias()[1]
        message.angular_velocity.z -= self._zero.bias()[2]
        if message.angular_velocity_covariance[0] >= 0:
            for axis, variance in enumerate(self._zero.variance()):
                message.angular_velocity_covariance[axis * 4] += variance
        self._topic.publish(message)
