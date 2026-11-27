"""Observations of configured ArUco markers in registered RGB-D images."""

from dataclasses import dataclass
from typing import final

import cv2
import numpy as np
import numpy.typing as npt

from robocell_sim.observation import Observation
from robocell_sim.projection import project


@final
@dataclass(frozen=True)
class Detection:
    """A metric observation together with its annotated camera preview."""

    _observation: Observation
    _image: npt.NDArray[np.uint8]

    def observation(self) -> Observation:
        return self._observation

    def image(self) -> npt.NDArray[np.uint8]:
        return self._image


def detected(
    image: npt.NDArray[np.uint8],
    depth: npt.NDArray[np.uint16],
    matrix: npt.NDArray[np.float64],
    dictionary: int,
    identifier: int,
    frame: str,
    stamp: float,
) -> Detection:
    """Select the configured marker, project its center and draw its preview."""
    corners, identifiers, _ = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(dictionary)
    ).detectMarkers(image)
    if identifiers is None:
        raise ValueError("Configured marker was not detected")
    selected = [
        (candidate, found)
        for candidate, found in zip(corners, identifiers, strict=True)
        if int(found[0]) == identifier
    ]
    if not selected:
        raise ValueError("Configured marker was not detected")
    corners, found = selected[0]
    preview = image.copy()
    cv2.aruco.drawDetectedMarkers(preview, [corners], found)
    observation = Observation(
        project(corners.reshape(4, 2).astype(np.float64), depth, matrix),
        frame,
        stamp,
        True,
    )
    return Detection(observation, preview)


def detect(
    image: npt.NDArray[np.uint8],
    depth: npt.NDArray[np.uint16],
    matrix: npt.NDArray[np.float64],
    dictionary: int,
    identifier: int,
    frame: str,
    stamp: float,
) -> Observation:
    """Return only the metric observation of the configured marker."""
    return detected(
        image, depth, matrix, dictionary, identifier, frame, stamp
    ).observation()
