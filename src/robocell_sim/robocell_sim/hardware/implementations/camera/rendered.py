"""Camera rendered by MuJoCo."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np
import numpy.typing as npt

from robocell_sim.hardware.interfaces.camera import Camera


@final
@dataclass(frozen=True)
class Rendered:
    """Camera that MuJoCo renders off screen from a camera in the model.

    The renderer fixes the image size and holds the drawing context, the
    data holds the state of the simulation to draw, and the camera names
    the color camera of the model.
    """

    _renderer: mujoco.Renderer
    _data: mujoco.MjData
    _camera: str

    def shot(self) -> npt.NDArray[np.uint8]:
        self._renderer.update_scene(self._data, camera=self._camera)
        return self._renderer.render()


@final
@dataclass(frozen=True)
class Paired(Camera):
    """Render color and depth at separate resolutions from one simulation state."""

    _color: Rendered
    _renderer: mujoco.Renderer
    _data: mujoco.MjData

    @override
    def frame(self) -> tuple[npt.NDArray[np.uint8], npt.NDArray[np.float32]]:
        picture = self._color.shot()
        self._renderer.update_scene(self._data, camera="camera_depth")
        self._renderer.enable_depth_rendering()
        try:
            return picture, self._renderer.render()
        finally:
            self._renderer.disable_depth_rendering()
