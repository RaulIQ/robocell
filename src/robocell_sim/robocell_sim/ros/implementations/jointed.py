"""Report that publishes the wheel joints by their encoders."""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import final, override

import mujoco
from rclpy.time import Time
from sensor_msgs.msg import JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.hardware.interfaces.wheel import Wheel
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Jointed(Report):
    """Report that publishes the angles of the wheels.

    The data holds the live state of the simulation, the wheels map each
    joint name of the URDF to the wheel
    whose encoder measures it, and the topic carries the angles as
    sensor_msgs/JointState stamped with the simulated time. The angles come
    from whole encoder ticks rather than the exact MuJoCo joints, so the
    message matches what the motor driver will later send over CAN.
    """

    _data: mujoco.MjData
    _wheels: Mapping[str, Wheel]
    _topic: Topic

    @override
    def publish(self) -> None:
        self._topic.publish(
            JointState(
                header=Header(
                    stamp=Time(
                        nanoseconds=round(self._data.time * 1e9)
                    ).to_msg()
                ),
                name=list(self._wheels),
                position=[wheel.angle() for wheel in self._wheels.values()],
            )
        )
