"""Dots that mark lidar echoes in the viewer."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np

from robocell_sim.display.interfaces.echoes import Echoes
from robocell_sim.display.interfaces.overlay import Overlay


@final
@dataclass(frozen=True)
class Dots(Overlay):
    """Small yellow spheres at the points where the lidar rays echoed.

    The echoes place the points in the world. Dots that do not fit into
    the scene are left out, so a full scene never overflows.
    """

    _echoes: Echoes

    @override
    def paint(self, scene: mujoco.MjvScene) -> None:
        for point in self._echoes.points()[: scene.maxgeom - scene.ngeom]:
            mujoco.mjv_initGeom(
                scene.geoms[scene.ngeom],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([0.015, 0.0, 0.0]),
                point,
                np.eye(3).ravel(),
                np.array([1.0, 0.85, 0.1, 1.0], dtype=np.float32),
            )
            scene.ngeom += 1
