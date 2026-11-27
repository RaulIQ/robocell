"""Visual target detection in registered color and depth images."""

from typing import Protocol

import numpy as np
import numpy.typing as npt

Detection = tuple[tuple[float, float, float], npt.NDArray[np.uint8]]


class DetectionSource(Protocol):
    """Locate a target in the camera frame and mark its preview."""

    def __call__(
        self,
        image: npt.NDArray[np.uint8],
        depth: npt.NDArray[np.uint16],
        matrix: npt.NDArray[np.float64],
    ) -> Detection:
        """Measure the target position and annotate the selected outline."""
        ...
