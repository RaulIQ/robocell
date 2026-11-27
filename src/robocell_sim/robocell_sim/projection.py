"""Metric target positions reconstructed from registered RGB-D images."""

import cv2
import numpy as np
import numpy.typing as npt


def project(
    corners: npt.NDArray[np.float64],
    depth: npt.NDArray[np.uint16],
    matrix: npt.NDArray[np.float64],
) -> tuple[float, float, float]:
    """Project the marker center through the median valid interior depth."""
    mask = np.zeros(depth.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(corners).astype(np.int32)], 1)
    interior = cv2.erode(mask, np.ones((3, 3), dtype=np.uint8))
    samples = depth[interior != 0]
    valid = samples[samples != 0]
    if valid.size == 0:
        raise ValueError("Marker requires valid depth")
    center = np.mean(corners, axis=0)
    distance = float(np.median(valid)) / 1000.0
    return (
        float((center[0] - matrix[0, 2]) * distance / matrix[0, 0]),
        float((center[1] - matrix[1, 2]) * distance / matrix[1, 1]),
        distance,
    )
