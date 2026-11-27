"""Tests for selecting and locating configured ArUco markers."""

from dataclasses import astuple

import cv2
import numpy as np
import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.detection import detect


def test_detection_selects_the_configured_marker_for_projection() -> None:
    """Only the selected marker supplies corners and depth to the projection."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = np.full((240, 280), 255, dtype=np.uint8)
    image[30:95, 20:85] = cv2.aruco.generateImageMarker(dictionary, 3, 65)
    image[80:145, 160:225] = cv2.aruco.generateImageMarker(dictionary, 7, 65)
    depth = np.full(image.shape, 1200, dtype=np.uint16)
    depth[80:145, 160:225] = 2300
    assert_that(
        astuple(
            detect(
                image,
                depth,
                np.array([[180.0, 0.0, 140.0], [0.0, 175.0, 120.0], [0.0, 0.0, 1.0]]),
                cv2.aruco.DICT_4X4_50,
                7,
                "camera_rgb_optical_frame",
                91.75,
            )
        ),
        equal_to(
            (
                ((192.0 - 140.0) * 2.3 / 180.0, (112.0 - 120.0) * 2.3 / 175.0, 2.3),
                "camera_rgb_optical_frame",
                91.75,
                True,
            )
        ),
        "the detector did not project the configured marker",
    )


def test_detection_rejects_an_image_without_the_configured_marker() -> None:
    """Other marker IDs cannot become the followed target."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = np.full((120, 160), 255, dtype=np.uint8)
    image[25:90, 45:110] = cv2.aruco.generateImageMarker(dictionary, 2, 65)
    with pytest.raises(ValueError) as error:
        detect(
            image,
            np.full(image.shape, 1500, dtype=np.uint16),
            np.eye(3),
            cv2.aruco.DICT_4X4_50,
            9,
            "camera_rgb_optical_frame",
            37.5,
        )
    assert_that(
        str(error.value),
        equal_to("Configured marker was not detected"),
        "the detector did not reject an unconfigured marker",
    )
