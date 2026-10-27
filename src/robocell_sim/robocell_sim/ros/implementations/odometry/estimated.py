"""Report that publishes the odometry of the robot."""

import math
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import (  # type: ignore[import-untyped]  # ROS generates message modules without types
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
)
from nav_msgs.msg import Odometry as Message  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.odometry import Odometry
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Estimated(Report):
    """Report that publishes where the robot reckons it is as nav_msgs/Odometry.

    The odometry gives the position and the heading, the header the stamp
    of the inputs and the frame the pose lies in, the child names the body
    the pose places, and the topic carries the message. The pose lies on
    the floor and turns only about z; the twist stays zero, since the
    odometry keeps no velocity.
    """

    _odometry: Odometry
    _header: Header
    _child: str
    _topic: Topic

    @override
    def publish(self) -> None:
        heading = self._odometry.heading()
        self._topic.publish(
            Message(
                header=self._header,
                child_frame_id=self._child,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(
                            x=self._odometry.position()[0],
                            y=self._odometry.position()[1],
                            z=0.0,
                        ),
                        orientation=Quaternion(
                            w=math.cos(heading / 2),
                            x=0.0,
                            y=0.0,
                            z=math.sin(heading / 2),
                        ),
                    )
                ),
            )
        )
