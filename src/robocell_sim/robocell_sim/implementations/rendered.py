"""Camera rendered by MuJoCo."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np
import numpy.typing as npt

from robocell_sim.interfaces.camera import Camera


@final
@dataclass(frozen=True)
class Rendered(Camera):
    """Camera that MuJoCo renders off screen from a camera in the model.

    The renderer fixes the image size and holds the drawing context, the
    data holds the state of the simulation to draw, and the camera names
    the camera of the model to look through.
    """

    _renderer: mujoco.Renderer
    _data: mujoco.MjData
    _camera: str

    @override
    def shot(self) -> npt.NDArray[np.uint8]:
        self._renderer.update_scene(self._data, camera=self._camera)
        return self._renderer.render()
