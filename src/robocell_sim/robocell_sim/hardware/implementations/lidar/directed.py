"""One clockwise ray through the current MuJoCo world."""

import math
from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np

from robocell_sim.hardware.interfaces.beam import Beam


@final
@dataclass(frozen=True)
class Directed(Beam):
    """Beam that measures each heading against the live physics state.

    Data holds the current scene, the body is the scanner's frame, and
    limits give the accepted distance interval in meters. No full-circle
    snapshot is cached, so each ray follows the moving scanner and scene.
    """

    _data: mujoco.MjData
    _body: str
    _limits: tuple[float, float]

    @override
    def distance(self, angle: float) -> float:
        body = self._data.model.body(self._body).id
        distance = mujoco.mj_ray(
            self._data.model,
            self._data,
            self._data.xpos[body].copy(),
            self._data.xmat[body].reshape(3, 3)
            @ np.array(
                [
                    math.cos(math.radians(angle)),
                    -math.sin(math.radians(angle)),
                    0.0,
                ]
            ),
            None,
            flg_static=True,
            bodyexclude=body,
            geomid=np.zeros(1, dtype=np.int32),
        )
        return (
            float(distance)
            if self._limits[0] <= distance <= self._limits[1]
            else math.inf
        )
