"""Report that publishes the footprint of the robot to TF."""

import math
from dataclasses import dataclass
from typing import final, override

import mujoco
from geometry_msgs.msg import (  # type: ignore[import-untyped]  # ROS generates message modules without types
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from rclpy.time import Time
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_msgs.msg import TFMessage  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Grounded(Report):
    """Report that places base_footprint in the odom frame by the true pose.

    This is a temporary bridge: the odometry of the robot will take its
    place and reckon the footprint from the wheels rather than from MuJoCo.
    The data holds the live state of the simulation, the body names the
    MuJoCo body above the footprint, and the topic carries the transform
    as tf2_msgs/TFMessage stamped with the simulated time. The footprint
    lies on the floor right below the body and keeps only its heading, so
    robot_state_publisher alone lifts base_link above it by the offset of
    the URDF.
    """

    _data: mujoco.MjData
    _body: str
    _topic: Topic

    @override
    def publish(self) -> None:
        heading = math.atan2(
            self._data.body(self._body).xmat[3],
            self._data.body(self._body).xmat[0],
        )
        self._topic.publish(
            TFMessage(
                transforms=[
                    TransformStamped(
                        header=Header(
                            stamp=Time(
                                nanoseconds=round(self._data.time * 1e9)
                            ).to_msg(),
                            frame_id="odom",
                        ),
                        child_frame_id="base_footprint",
                        transform=Transform(
                            translation=Vector3(
                                x=float(self._data.body(self._body).xpos[0]),
                                y=float(self._data.body(self._body).xpos[1]),
                                z=0.0,
                            ),
                            rotation=Quaternion(
                                w=math.cos(heading / 2),
                                x=0.0,
                                y=0.0,
                                z=math.sin(heading / 2),
                            ),
                        ),
                    )
                ]
            )
        )
