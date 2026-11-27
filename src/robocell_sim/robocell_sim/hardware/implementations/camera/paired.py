"""A synchronized color and depth camera."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np
import numpy.typing as npt

from robocell_sim.hardware.interfaces.camera import Camera
from robocell_sim.hardware.interfaces.still import Still


@final
@dataclass(frozen=True)
class Paired(Camera):
    """Render color and depth at separate resolutions from one state."""

    _color: Still
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
