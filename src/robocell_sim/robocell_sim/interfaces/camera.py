"""Camera of a robot."""

from typing import Protocol

import numpy as np
import numpy.typing as npt


class Camera(Protocol):
    """Camera that takes a picture of the world as it is right now."""

    def shot(self) -> npt.NDArray[np.uint8]:
        """RGB image as rows of pixels, each with three bytes."""
