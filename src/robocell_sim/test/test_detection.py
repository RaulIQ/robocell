"""Tests for selecting and locating configured ArUco markers."""

from dataclasses import astuple

import cv2
import numpy as np
import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.detection import detect, detected


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


def test_detection_annotates_only_the_selected_marker() -> None:
    """The preview outlines the followed ID without changing other markers."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    gray = np.full((150, 240), 255, dtype=np.uint8)
    gray[20:85, 20:85] = cv2.aruco.generateImageMarker(dictionary, 3, 65)
    gray[55:120, 150:215] = cv2.aruco.generateImageMarker(dictionary, 7, 65)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    result = detected(
        image,
        np.full(gray.shape, 2300, dtype=np.uint16),
        np.array([[180.0, 0.0, 120.0], [0.0, 175.0, 75.0], [0.0, 0.0, 1.0]]),
        cv2.aruco.DICT_4X4_50,
        7,
        "camera_rgb_optical_frame",
        91.75,
    )
    assert_that(
        (
            bool(np.any(result.image()[55:120, 150:215] != image[55:120, 150:215])),
            bool(np.any(result.image()[20:85, 20:85] != image[20:85, 20:85])),
            result.observation().position()[2],
        ),
        equal_to((True, False, 2.3)),
        "the preview did not annotate only the configured marker",
    )


def test_detection_tracks_a_moving_marker() -> None:
    """A later frame supplies its new image position and metric depth."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 7, 65)
    first = np.full((130, 170), 255, dtype=np.uint8)
    first[20:85, 20:85] = marker
    second = np.full((130, 170), 255, dtype=np.uint8)
    second[35:100, 55:120] = marker
    matrix = np.array([[100.0, 0.0, 80.0], [0.0, 110.0, 60.0], [0.0, 0.0, 1.0]])
    assert_that(
        (
            detect(
                first,
                np.full(first.shape, 1200, dtype=np.uint16),
                matrix,
                cv2.aruco.DICT_4X4_50,
                7,
                "camera_rgb_optical_frame",
                10.0,
            ).position(),
            detect(
                second,
                np.full(second.shape, 1700, dtype=np.uint16),
                matrix,
                cv2.aruco.DICT_4X4_50,
                7,
                "camera_rgb_optical_frame",
                10.1,
            ).position(),
        ),
        equal_to(
            (
                ((52.0 - 80.0) * 1.2 / 100.0, (52.0 - 60.0) * 1.2 / 110.0, 1.2),
                ((87.0 - 80.0) * 1.7 / 100.0, (67.0 - 60.0) * 1.7 / 110.0, 1.7),
            )
        ),
        "the detector did not track the moving marker",
    )


def test_detection_rejects_an_occluded_marker() -> None:
    """A partly hidden marker cannot supply a target observation."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = np.full((120, 160), 255, dtype=np.uint8)
    image[25:90, 45:110] = cv2.aruco.generateImageMarker(dictionary, 0, 65)
    image[25:90, 85:110] = 255
    with pytest.raises(ValueError) as error:
        detect(
            image,
            np.full(image.shape, 1500, dtype=np.uint16),
            np.eye(3),
            cv2.aruco.DICT_4X4_50,
            0,
            "camera_rgb_optical_frame",
            37.5,
        )
    assert_that(
        str(error.value),
        equal_to("Configured marker was not detected"),
        "the detector did not reject an occluded marker",
    )
