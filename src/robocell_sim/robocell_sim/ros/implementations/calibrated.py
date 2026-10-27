"""Report that publishes the calibration of the camera."""

from dataclasses import dataclass
from typing import final, override

import mujoco
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.intrinsics import Intrinsics
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Calibrated(Report):
    """Report that publishes how the camera projects the world onto pixels.

    The data holds the live state of the simulation, the intrinsics give
    the camera matrix, the size is the width and the height of the image
    in pixels, and the topic carries them as sensor_msgs/CameraInfo in the
    camera_rgb_optical_frame of the URDF, stamped with the simulated time.
    The camera is an ideal pinhole, so its lens bends nothing and the
    projection is the camera matrix with no shift.
    """

    _data: mujoco.MjData
    _intrinsics: Intrinsics
    _size: tuple[int, int]
    _topic: Topic

    @override
    def publish(self) -> None:
        matrix = self._intrinsics.matrix()
        self._topic.publish(
            CameraInfo(
                header=Header(
                    stamp=Time(
                        nanoseconds=round(self._data.time * 1e9)
                    ).to_msg(),
                    frame_id="camera_rgb_optical_frame",
                ),
                width=self._size[0],
                height=self._size[1],
                distortion_model="plumb_bob",
                d=[0.0] * 5,
                k=[value for row in matrix for value in row],
                r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                p=[value for row in matrix for value in (*row, 0.0)],
            )
        )
