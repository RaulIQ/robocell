"""Tests for converting synchronized ROS RGB-D frames into observations."""

import cv2
import numpy as np
from cv_bridge import CvBridge
from hamcrest import assert_that, equal_to
from rclpy.time import Time
from robocell_sim.robot.implementations.detection.detection import Detected
from robocell_sim.robot.implementations.detection.projection import Projected
from robocell_sim.ros.implementations.pose.observed import Observed
from robocell_sim.ros.messages import CameraInfo, Header


def pixels() -> np.ndarray[tuple[int, int], np.dtype[np.uint8]]:
    """Build a color fixture with a single off-center marker."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 7, 65)
    pixels = np.full((120, 160), 255, dtype=np.uint8)
    pixels[25:90, 45:110] = marker
    return pixels


def test_observed_preserves_frame_and_stamp() -> None:
    """The adapter preserves calibration and RGB acquisition time."""
    bridge = CvBridge()
    gray = pixels()
    image = bridge.cv2_to_imgmsg(
        cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB).astype(np.uint8), "rgb8"
    )
    image.header = Header(
        stamp=Time(nanoseconds=91_750_000_000).to_msg(),
        frame_id="camera_rgb_optical_frame",
    )
    depth = bridge.cv2_to_imgmsg(
        np.full(gray.shape, 2300, dtype=np.uint16), "16UC1"
    )
    calibration = CameraInfo(
        header=image.header,
        k=[180.0, 0.0, 80.0, 0.0, 175.0, 60.0, 0.0, 0.0, 1.0],
    )
    observation, _ = Observed(
        Detected(cv2.aruco.DICT_4X4_50, 7, Projected((3, 3))), bridge
    )(image, depth, calibration)
    assert_that(
        observation.reading(),
        equal_to(
            (
                ((77.0 - 80.0) * 2.3 / 180.0, (57.0 - 60.0) * 2.3 / 175.0, 2.3),
                "camera_rgb_optical_frame",
                91.75,
                True,
            )
        ),
        "the ROS adapter did not preserve the detected observation",
    )
