"""Camera that sees one flat color."""

from dataclasses import dataclass
from typing import final, override

import numpy as np
import numpy.typing as npt
from robocell_sim.hardware.interfaces.camera import Camera


@final
@dataclass(frozen=True)
class Painted(Camera):
    """Camera whose every pixel shows the same color.

    The color holds the red, green and blue bytes of each pixel, and the
    size is the width and the height of the image in pixels, so a test
    knows every byte of a shot without rendering a world.
    """

    _color: tuple[int, int, int]
    _size: tuple[int, int]

    @override
    def shot(self) -> npt.NDArray[np.uint8]:
        return np.full((self._size[1], self._size[0], 3), self._color, np.uint8)
