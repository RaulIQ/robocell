"""Lidar that casts MuJoCo rays."""

import math
from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np

from robocell_sim.interfaces.lidar import Lidar


@final
@dataclass(frozen=True)
class Raycast(Lidar):
    """Lidar that casts one level ray per degree from a body of the model.

    The model and the data hold the world and its live state, the body
    names the scanner, whose own geoms the rays skip, and the limits are
    the shortest and the longest range in meters that the lidar reports.
    The rays turn with the body, start at its origin, and count angles
    counterclockwise from ahead, as ROS does.
    """

    _model: mujoco.MjModel
    _data: mujoco.MjData
    _body: str
    _limits: tuple[float, float]

    @override
    def ranges(self) -> tuple[float, ...]:
        body = self._model.body(self._body).id
        distance = np.zeros(360)
        mujoco.mj_multiRay(
            self._model,
            self._data,
            self._data.xpos[body].copy(),
            (
                np.column_stack(
                    (
                        np.cos(np.radians(np.arange(360))),
                        np.sin(np.radians(np.arange(360))),
                        np.zeros(360),
                    )
                )
                @ self._data.xmat[body].reshape(3, 3).T
            ).ravel(),
            None,
            flg_static=True,
            bodyexclude=body,
            geomid=np.zeros(360, dtype=np.int32),
            dist=distance,
            normal=None,
            nray=360,
            cutoff=self._limits[1],
        )
        return tuple(
            float(echo)
            if self._limits[0] <= echo <= self._limits[1]
            else math.inf
            for echo in distance
        )
