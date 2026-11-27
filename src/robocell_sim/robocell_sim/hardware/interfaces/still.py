"""The color camera view of the current scene."""

from typing import Protocol

import numpy as np
import numpy.typing as npt


class Still(Protocol):
    """Capture a color image from the current scene."""

    def shot(self) -> npt.NDArray[np.uint8]:
        """Capture the RGB pixels."""
        ...
