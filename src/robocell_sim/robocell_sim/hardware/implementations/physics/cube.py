"""Motion of the kinematic target in the MuJoCo workshop."""

import math
from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.prop import Prop


@final
@dataclass(frozen=True)
class Cube(Prop):
    """A movable target independent of the wheel drives."""

    _data: mujoco.MjData
    _index: int

    @override
    def place(self) -> None:
        """Raise the target from storage beneath the workshop floor."""
        self._data.mocap_pos[self._index, 2] = 0.05

    @override
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

    @override
    def pose(self) -> tuple[float, float, float]:
        """Read the position and heading in MuJoCo world coordinates."""
        position = self._data.mocap_pos[self._index]
        rotation = self._data.mocap_quat[self._index]
        return (
            float(position[0]),
            float(position[1]),
            2 * math.atan2(rotation[3], rotation[0]),
        )
