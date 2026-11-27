"""The input and outputs of camera target tracking."""

from typing import Protocol

import numpy as np
import numpy.typing as npt

from robocell_sim.robot.interfaces.target_observation import Reading
from robocell_sim.ros.messages import CameraInfo, Image


class Preview(Protocol):
    """Process observations and report the outcome beside the image."""

    def measure(
        self, image: Image, depth: Image, calibration: CameraInfo
    ) -> None:
        """Read a fresh synchronized frame."""
        ...

    def reject(self, image: Image, reason: str) -> None:
        """Report why the unaltered image could not be used."""
        ...

    def show(
        self, image: Image, reading: Reading, pixels: npt.NDArray[np.uint8]
    ) -> None:
        """Publish a preview and the measured position."""
        ...
