"""Report that publishes the odometry of the robot."""

import math
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import (  # type: ignore[import-untyped]  # ROS generates message modules without types
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry as Message  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.travel import Travel
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Estimated(Report):
    """Report that publishes the odometry of the wheels as nav_msgs/Odometry.

    The travel gives the position, the heading and the velocity, the
    variances spread the forward speed and the yaw rate, the frames hold
    the header with the stamp of the inputs and the frame the pose lies in
    and then the body the pose places, and the topic carries the message.
    The pose lies on the floor and turns only about z, and the twist moves
    only along x and turns only about z, as a differential drive does.
    """

    _travel: Travel
    _variances: tuple[float, float]
    _frames: tuple[Header, str]
    _topic: Topic

    @override
    def publish(self) -> None:
        heading = self._travel.heading()
        self._topic.publish(
            Message(
                header=self._frames[0],
                child_frame_id=self._frames[1],
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(
                            x=self._travel.position()[0],
                            y=self._travel.position()[1],
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
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(
                            x=self._travel.velocity()[0], y=0.0, z=0.0
                        ),
                        angular=Vector3(
                            x=0.0, y=0.0, z=self._travel.velocity()[1]
                        ),
                    ),
                    covariance=[self._variances[0]]
                    + [0.0] * 34
                    + [self._variances[1]],
                ),
            )
        )
