"""Report that publishes the picture of the camera."""

from dataclasses import dataclass
from typing import final, override

import mujoco
from rclpy.time import Time
from sensor_msgs.msg import Image  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.camera import Camera
from robocell_sim.ros.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Filmed(Report):
    """Report that publishes what the camera of the robot sees.

    The data holds the live state of the simulation, the camera takes the
    picture, and the topic carries it as an rgb8 sensor_msgs/Image in the
    camera_rgb_optical_frame of the URDF, stamped with the simulated time.
    """

    _data: mujoco.MjData
    _camera: Camera
    _topic: Topic

    @override
    def publish(self) -> None:
        shot = self._camera.shot()
        self._topic.publish(
            Image(
                header=Header(
                    stamp=Time(
                        nanoseconds=round(self._data.time * 1e9)
                    ).to_msg(),
                    frame_id="camera_rgb_optical_frame",
                ),
                height=shot.shape[0],
                width=shot.shape[1],
                encoding="rgb8",
                step=shot.shape[1] * 3,
                data=shot.tobytes(),
            )
        )
