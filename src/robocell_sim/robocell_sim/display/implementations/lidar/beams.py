"""Lines that draw lidar rays in the viewer."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np

from robocell_sim.display.interfaces.echoes import Echoes
from robocell_sim.display.interfaces.overlay import Overlay


@final
@dataclass(frozen=True)
class Beams(Overlay):
    """Thin red lines from the scanner to every point where a ray echoed.

    The echoes give the start of the rays and their ends. Lines that do
    not fit into the scene are left out, so a full scene never overflows.
    """

    _echoes: Echoes

    @override
    def paint(self, scene: mujoco.MjvScene) -> None:
        for point in self._echoes.points()[: scene.maxgeom - scene.ngeom]:
            mujoco.mjv_initGeom(
                scene.geoms[scene.ngeom],
                mujoco.mjtGeom.mjGEOM_LINE,
                np.zeros(3),
                np.zeros(3),
                np.eye(3).ravel(),
                np.array([1.0, 0.2, 0.2, 0.8], dtype=np.float32),
            )
            mujoco.mjv_connector(
                scene.geoms[scene.ngeom],
                mujoco.mjtGeom.mjGEOM_LINE,
                2,
                self._echoes.origin(),
                point,
            )
            scene.ngeom += 1
