"""Camera of a robot."""

from typing import Protocol

import numpy as np
import numpy.typing as npt


class Camera(Protocol):
    """Camera that takes a picture of the world as it is right now."""

    def frame(self) -> tuple[npt.NDArray[np.uint8], npt.NDArray[np.float32]]:
        """RGB and optical-axis depth in meters from one simulation state."""
        ...
