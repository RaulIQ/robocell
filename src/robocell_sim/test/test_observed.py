"""Tests for converting synchronized ROS RGB-D frames into observations."""

import cv2
import numpy as np
from cv_bridge import CvBridge
from hamcrest import assert_that, equal_to
from rclpy.time import Time
from robocell_sim.ros.implementations.pose.observed import Observed
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


def test_observed_converts_ros_images_and_calibration_for_detection() -> None:
    """The ROS adapter preserves the camera frame and image timestamp."""
    bridge = CvBridge()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 7, 65)
    pixels = np.full((120, 160), 255, dtype=np.uint8)
    pixels[25:90, 45:110] = marker
    image = bridge.cv2_to_imgmsg(cv2.cvtColor(pixels, cv2.COLOR_GRAY2RGB), "rgb8")
    image.header = Header(
        stamp=Time(nanoseconds=91_750_000_000).to_msg(),
        frame_id="camera_rgb_optical_frame",
    )
    depth = bridge.cv2_to_imgmsg(np.full(pixels.shape, 2300, dtype=np.uint16), "16UC1")
    depth.header = image.header
    calibration = CameraInfo(
        header=image.header,
        k=[180.0, 0.0, 80.0, 0.0, 175.0, 60.0, 0.0, 0.0, 1.0],
    )
    observation = Observed(7, cv2.aruco.DICT_4X4_50, bridge).observe(
        image, depth, calibration
    )
    assert_that(
        (
            observation.position(),
            observation.frame(),
            observation.stamp(),
            observation.valid(),
        ),
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
