"""Intrinsics of an ideal pinhole camera."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.interfaces.intrinsics import Intrinsics


@final
@dataclass(frozen=True)
class Pinhole(Intrinsics):
    """Intrinsics of a camera with square pixels and no lens distortion.

    The fovy is the vertical field of view in radians, and the width and
    the height are the image size in pixels. MuJoCo renders through such
    a camera, so its intrinsics follow from the field of view alone.
    """

    _fovy: float
    _width: int
    _height: int

    @override
    def matrix(
        self,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]:
        return (
            (
                self._height / 2 / math.tan(self._fovy / 2),
                0.0,
                self._width / 2,
            ),
            (
                0.0,
                self._height / 2 / math.tan(self._fovy / 2),
                self._height / 2,
            ),
            (0.0, 0.0, 1.0),
        )
