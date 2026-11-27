"""Observations of configured ArUco markers in registered RGB-D images."""

import cv2
import numpy as np
import numpy.typing as npt

from robocell_sim.observation import Observation
from robocell_sim.projection import project


def detect(
    image: npt.NDArray[np.uint8],
    depth: npt.NDArray[np.uint16],
    matrix: npt.NDArray[np.float64],
    dictionary: int,
    identifier: int,
    frame: str,
    stamp: float,
) -> Observation:
    """Select the configured marker and project its center into metric space."""
    corners, identifiers, _ = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(dictionary)
    ).detectMarkers(image)
    if identifiers is None:
        raise ValueError("Configured marker was not detected")
    selected = [
        candidate.reshape(4, 2).astype(np.float64)
        for candidate, found in zip(corners, identifiers, strict=True)
        if int(found[0]) == identifier
    ]
    if not selected:
        raise ValueError("Configured marker was not detected")
    return Observation(project(selected[0], depth, matrix), frame, stamp, True)
