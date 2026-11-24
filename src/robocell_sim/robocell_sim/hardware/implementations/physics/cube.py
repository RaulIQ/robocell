"""Motion of the kinematic target in the MuJoCo workshop."""

import math
from dataclasses import dataclass
from typing import final

import mujoco


@final
@dataclass(frozen=True)
class Cube:
    """A movable target independent of the wheel drives."""

    _data: mujoco.MjData
    _index: int

    def place(self) -> None:
        """Raise the target from storage beneath the workshop floor."""
        self._data.mocap_pos[self._index, 2] = 0.05

    def move(self, velocity: tuple[float, float], elapsed: float) -> None:
        """Advance a planar command without crossing the workshop walls."""
        linear, angular = velocity
        position = self._data.mocap_pos[self._index]
        rotation = self._data.mocap_quat[self._index]
        heading = 2 * math.atan2(rotation[3], rotation[0])
        middle = heading + angular * elapsed / 2
        position[0] = max(
            -2.85, min(2.85, position[0] + linear * elapsed * math.cos(middle))
        )
        position[1] = max(
            -2.85, min(2.85, position[1] + linear * elapsed * math.sin(middle))
        )
        heading += angular * elapsed
        rotation[:] = (math.cos(heading / 2), 0, 0, math.sin(heading / 2))
