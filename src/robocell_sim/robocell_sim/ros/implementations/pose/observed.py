"""Observations decoded from synchronized ROS RGB-D messages."""

from dataclasses import dataclass
from typing import final

import numpy as np
import numpy.typing as npt
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.observation import Observation
from robocell_sim.robot.interfaces.detection import DetectionSource


@final
@dataclass(frozen=True)
class Observed:
    """Convert registered camera messages into the common target observation."""

    _detected: DetectionSource
    _bridge: CvBridge

    def __call__(
        self, image: Image, depth: Image, calibration: CameraInfo
    ) -> tuple[Observation, npt.NDArray[np.uint8]]:
        """Decode registered frames and preserve their acquisition metadata."""
        position, preview = self._detected(
            self._bridge.imgmsg_to_cv2(image, "rgb8"),
            self._bridge.imgmsg_to_cv2(depth, "16UC1"),
            np.asarray(calibration.k, dtype=np.float64).reshape(3, 3),
        )
        return Observation(
            position,
            calibration.header.frame_id,
            Time.from_msg(image.header.stamp).nanoseconds / 1e9,
            _valid=True,
        ), preview
