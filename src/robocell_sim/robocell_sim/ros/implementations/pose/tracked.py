"""Report that publishes the true pose of a body."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np
from geometry_msgs.msg import (  # type: ignore[import-untyped]  # ROS generates message modules without types
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.time import Time
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Tracked(Report):
    """Report that publishes where a body truly is.

    The data holds the live state of the simulation, the body names the
    MuJoCo body to follow, and the topic carries it as nav_msgs/Odometry:
    the pose in the world frame and, as ROS expects, the twist in the frame
    of the body, stamped with the simulated time.
    """

    _data: mujoco.MjData
    _body: str
    _topic: Topic

    @override
    def publish(self) -> None:
        body = self._data.body(self._body)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            self._data.model,
            self._data,
            mujoco.mjtObj.mjOBJ_XBODY,
            self._data.model.body(self._body).id,
            velocity,
            1,
        )
        self._topic.publish(
            Odometry(
                header=Header(
                    stamp=Time(
                        nanoseconds=round(self._data.time * 1e9)
                    ).to_msg(),
                    frame_id="world",
                ),
                child_frame_id=self._body,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(
                            x=float(body.xpos[0]),
                            y=float(body.xpos[1]),
                            z=float(body.xpos[2]),
                        ),
                        orientation=Quaternion(
                            w=float(body.xquat[0]),
                            x=float(body.xquat[1]),
                            y=float(body.xquat[2]),
                            z=float(body.xquat[3]),
                        ),
                    )
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(
                            x=float(velocity[3]),
                            y=float(velocity[4]),
                            z=float(velocity[5]),
                        ),
                        angular=Vector3(
                            x=float(velocity[0]),
                            y=float(velocity[1]),
                            z=float(velocity[2]),
                        ),
                    )
                ),
            )
        )
