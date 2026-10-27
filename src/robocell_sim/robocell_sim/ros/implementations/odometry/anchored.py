"""Report that places the body by the odometry in TF."""

import math
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import (  # type: ignore[import-untyped]  # ROS generates message modules without types
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_msgs.msg import TFMessage  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.odometry import Odometry
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Anchored(Report):
    """Report that places the body where the odometry reckons it.

    The odometry gives the position and the heading, the header the stamp
    of the inputs and the parent frame, odom, the child names the body,
    base_footprint, and the topic carries the transform as
    tf2_msgs/TFMessage. The body lies on the floor and turns only about z,
    so robot_state_publisher alone lifts base_link above it.
    """

    _odometry: Odometry
    _header: Header
    _child: str
    _topic: Topic

    @override
    def publish(self) -> None:
        heading = self._odometry.heading()
        self._topic.publish(
            TFMessage(
                transforms=[
                    TransformStamped(
                        header=self._header,
                        child_frame_id=self._child,
                        transform=Transform(
                            translation=Vector3(
                                x=self._odometry.position()[0],
                                y=self._odometry.position()[1],
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
