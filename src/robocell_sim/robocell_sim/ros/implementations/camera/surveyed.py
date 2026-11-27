"""Synchronized camera frames submitted to target detection."""

from collections.abc import Callable
from dataclasses import dataclass
from typing import final, override

import numpy as np
import numpy.typing as npt
from cv_bridge import CvBridge
from rclpy.time import Time

from robocell_sim.robot.implementations.detection.missing_depth import (
    MissingDepthError,
)
from robocell_sim.robot.interfaces.target_observation import Reading
from robocell_sim.ros.implementations.pose.observed import Observed
from robocell_sim.ros.interfaces.preview import Preview
from robocell_sim.ros.interfaces.topic import Topic
from robocell_sim.ros.messages import CameraInfo, Image, Point, PoseStamped


def pose(
    image: Image, source: str, position: tuple[float, float, float]
) -> PoseStamped:
    """Build the marker pose in the original camera frame."""
    goal = PoseStamped()
    goal.header.stamp = image.header.stamp
    goal.header.frame_id = source
    goal.pose.position = Point(x=position[0], y=position[1], z=position[2])
    goal.pose.orientation.w = 1.0
    return goal


@final
@dataclass(frozen=True)
class Surveyed(Preview):
    """Detect a fresh marker and publish a diagnostic preview."""

    _recent: Callable[[int], bool]
    _observed: Observed
    _preview: tuple[CvBridge, Topic]
    _output: tuple[Callable[[PoseStamped], None], Callable[[str], None]]

    def __call__(
        self, image: Image, depth: Image, calibration: CameraInfo
    ) -> None:
        """Process a synchronized RGB-D frame at its acquisition timestamp."""
        if not self._recent(Time.from_msg(image.header.stamp).nanoseconds):
            self.reject(image, "stale")
            return
        self.measure(image, depth, calibration)

    @override
    def measure(
        self, image: Image, depth: Image, calibration: CameraInfo
    ) -> None:
        try:
            observation, pixels = self._observed(image, depth, calibration)
        except MissingDepthError:
            self.reject(image, "invalid_depth")
        except ValueError:
            self.reject(image, "lost")
        else:
            self.show(image, observation.reading(), pixels)

    @override
    def reject(self, image: Image, reason: str) -> None:
        self._output[1](reason)
        self._preview[1].publish(image)

    @override
    def show(
        self, image: Image, reading: Reading, pixels: npt.NDArray[np.uint8]
    ) -> None:
        position, source, _, valid = reading
        if not valid:
            self.reject(image, "lost")
            return
        frame = self._preview[0].cv2_to_imgmsg(pixels, "rgb8")
        frame.header = image.header
        self._preview[1].publish(frame)
        self._output[0](pose(image, source, position))
