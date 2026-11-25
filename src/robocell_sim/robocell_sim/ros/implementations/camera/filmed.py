"""Report that publishes synchronized D415 color and depth images."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import Image  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.camera import Camera
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Filmed(Report):
    """Publish VGA color and depth in their respective optical frames."""

    _data: mujoco.MjData
    _camera: Camera
    _topic: Topic
    _depth: Topic

    @override
    def publish(self) -> None:
        picture, depth = self._camera.frame()
        stamp = Time(nanoseconds=round(self._data.time * 1e9)).to_msg()
        self._topic.publish(
            Image(
                header=Header(stamp=stamp, frame_id="camera_rgb_optical_frame"),
                height=picture.shape[0],
                width=picture.shape[1],
                encoding="rgb8",
                step=picture.shape[1] * 3,
                data=picture.tobytes(),
            )
        )
        self._depth.publish(
            Image(
                header=Header(stamp=stamp, frame_id="camera_depth_optical_frame"),
                height=depth.shape[0],
                width=depth.shape[1],
                encoding="16UC1",
                step=depth.shape[1] * 2,
                data=np.rint(
                    np.where((depth >= 0.310) & (depth <= 65.535), depth, 0)
                    * 1000
                ).astype("<u2").tobytes(),
            )
        )
