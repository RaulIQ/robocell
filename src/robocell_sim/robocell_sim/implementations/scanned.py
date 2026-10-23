"""Echoes of a lidar scan in world axes."""

import math
from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np
import numpy.typing as npt

from robocell_sim.interfaces.echoes import Echoes


@final
@dataclass(frozen=True)
class Scanned(Echoes):
    """Echoes that a scan of ranges leaves around the scanner right now.

    The data holds the live state of the simulation, the body names the
    scanner, and the scan holds ranges in meters, one per degree
    counterclockwise from ahead, with infinity where nothing echoed. The
    points follow the pose of the scanner at the moment they are read.
    """

    _data: mujoco.MjData
    _body: str
    _scan: tuple[float, ...]

    @override
    def origin(self) -> npt.NDArray[np.float64]:
        return np.array(self._data.body(self._body).xpos, dtype=np.float64)

    @override
    def points(self) -> list[npt.NDArray[np.float64]]:
        return [
            self._data.body(self._body).xpos
            + self._data.body(self._body).xmat.reshape(3, 3)
            @ np.array(
                [
                    distance * math.cos(math.radians(index)),
                    distance * math.sin(math.radians(index)),
                    0.0,
                ]
            )
            for index, distance in enumerate(self._scan)
            if math.isfinite(distance)
        ]
