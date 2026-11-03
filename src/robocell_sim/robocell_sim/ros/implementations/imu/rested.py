"""Stationary evidence independent of the simulated ground truth."""

import math
from dataclasses import dataclass
from typing import final, override

from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.standstill import Standstill

_SKEW = -0.02
_GRAVITY = 9.81
_TOLERANCE = 0.5


@final
@dataclass(frozen=True)
class Rested(Standstill):
    """Wheel standstill corroborated by fresh feedback and gravity.

    Limits hold maximum wheel speed and feedback age. Up to 20 ms of clock
    skew accommodates the independent wheel acquisition and IMU stamps.
    """

    _joints: JointState
    _message: Imu
    _limits: tuple[float, float]
    _names: tuple[str, str]

    @override
    def still(self) -> bool:
        if len(self._joints.name) != len(self._joints.velocity):
            return False
        speeds = dict(
            zip(self._joints.name, self._joints.velocity, strict=True)
        )
        return (
            all(
                name in speeds
                and math.isfinite(speeds[name])
                and abs(speeds[name]) <= self._limits[0]
                for name in self._names
            )
            and _SKEW
            <= (
                Time.from_msg(self._message.header.stamp).nanoseconds
                - Time.from_msg(self._joints.header.stamp).nanoseconds
            )
            * 1e-9
            <= self._limits[1]
            and abs(
                math.hypot(
                    self._message.linear_acceleration.x,
                    self._message.linear_acceleration.y,
                    self._message.linear_acceleration.z,
                )
                - _GRAVITY
            )
            <= _TOLERANCE
        )
