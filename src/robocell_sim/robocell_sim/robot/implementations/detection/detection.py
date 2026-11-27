"""Locate configured ArUco markers in registered RGB-D images."""

from dataclasses import dataclass
from typing import final, override

import cv2
import numpy as np
import numpy.typing as npt

from robocell_sim.robot.interfaces.detection import Detection, DetectionSource
from robocell_sim.robot.interfaces.projection import Projection


@final
@dataclass(frozen=True)
class Detected(DetectionSource):
    """Locate and outline a configured marker using calibrated depth."""

    _dictionary: int
    _identifier: int
    _projection: Projection

    @override
    def __call__(
        self,
        image: npt.NDArray[np.uint8],
        depth: npt.NDArray[np.uint16],
        matrix: npt.NDArray[np.float64],
    ) -> Detection:
        corners, identifiers, _ = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(self._dictionary)
        ).detectMarkers(image)
        matches = [
            (outline, identifier)
            for outline, identifier in zip(
                corners, () if identifiers is None else identifiers, strict=True
            )
            if int(identifier[0]) == self._identifier
        ]
        if not matches:
            error = "Configured marker was not detected"
            raise ValueError(error)
        outline, identifier = matches[0]
        position = self._projection(
            outline.reshape(4, 2).astype(np.float64), depth, matrix
        )
        preview = image.copy()
        cv2.aruco.drawDetectedMarkers(preview, [outline], identifier)
        return position, preview
