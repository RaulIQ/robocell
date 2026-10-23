"""Video of the robot camera."""

from typing import Protocol

import numpy as np
import numpy.typing as npt


class Video(Protocol):
    """Video that films the world through the robot camera as it changes."""

    def record(self, state: npt.NDArray[np.float64]) -> None:
        """Film the world at these joint positions of the simulation."""

    def show(self) -> None:
        """Show the video in a window until the window is closed."""
