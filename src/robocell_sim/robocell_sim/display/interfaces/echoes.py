"""Echoes of a lidar scan."""

from typing import Protocol

import numpy as np
import numpy.typing as npt


class Echoes(Protocol):
    """Echoes of one lidar scan placed in the world."""

    def origin(self) -> npt.NDArray[np.float64]:
        """Point in world axes where every ray of the scan starts."""
        ...

    def points(self) -> list[npt.NDArray[np.float64]]:
        """Points in world axes where the rays echoed, in scan order."""
        ...
