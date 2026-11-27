"""Observations decoded from synchronized ROS RGB-D messages."""

from dataclasses import dataclass
from typing import final

import numpy as np
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.detection import detect
from robocell_sim.observation import Observation


@final
@dataclass(frozen=True)
class Observed:
    """Convert registered camera messages into the common target observation."""

    _identifier: int
    _dictionary: int
    _bridge: CvBridge

    def observe(
        self, image: Image, depth: Image, calibration: CameraInfo
    ) -> Observation:
        return detect(
            self._bridge.imgmsg_to_cv2(image, "rgb8"),
            self._bridge.imgmsg_to_cv2(depth, "16UC1"),
            np.asarray(calibration.k, dtype=np.float64).reshape(3, 3),
            self._dictionary,
            self._identifier,
            calibration.header.frame_id,
            Time.from_msg(image.header.stamp).nanoseconds / 1e9,
        )
