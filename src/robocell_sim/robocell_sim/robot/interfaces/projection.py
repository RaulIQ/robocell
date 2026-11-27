"""Projection of a marked region in a registered depth image."""

from typing import Protocol

import numpy as np
import numpy.typing as npt


class Projection(Protocol):
    """Recover a metric camera position from an image region."""

    def __call__(
        self,
        corners: npt.NDArray[np.float64],
        depth: npt.NDArray[np.uint16],
        matrix: npt.NDArray[np.float64],
    ) -> tuple[float, float, float]:
        """Project the marked region through its calibrated depth."""
        ...
