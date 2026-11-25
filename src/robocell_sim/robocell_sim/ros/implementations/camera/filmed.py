"""Report that publishes aligned color and depth images."""

from dataclasses import dataclass
from typing import final, override

import mujoco
from rclpy.time import Time
from sensor_msgs.msg import Image  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.camera import Camera
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Filmed(Report):
    """Publish color and depth in the camera frame at the same simulated time."""

    _data: mujoco.MjData
    _camera: Camera
    _topic: Topic
    _depth: Topic

    @override
    def publish(self) -> None:
        picture, depth = self._camera.frame()
        header = Header(
            stamp=Time(nanoseconds=round(self._data.time * 1e9)).to_msg(),
            frame_id="camera_rgb_optical_frame",
        )
        self._topic.publish(
            Image(
                header=header,
                height=picture.shape[0],
                width=picture.shape[1],
                encoding="rgb8",
                step=picture.shape[1] * 3,
                data=picture.tobytes(),
            )
        )
        self._depth.publish(
            Image(
                header=header,
                height=depth.shape[0],
                width=depth.shape[1],
                encoding="32FC1",
                step=depth.shape[1] * 4,
                data=depth.astype("<f4", copy=False).tobytes(),
            )
        )
