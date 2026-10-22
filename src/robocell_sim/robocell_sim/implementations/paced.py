"""Physics that step up to a moment in time."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.interfaces.physics import Physics


@final
@dataclass(frozen=True)
class Paced(Physics):
    """MuJoCo physics that take as many steps as a moment in time needs.

    The model fixes the length of one step, and the data holds the live
    state of the simulation, whose clock never runs more than one step
    past the moment it was asked to reach.
    """

    _model: mujoco.MjModel
    _data: mujoco.MjData

    @override
    def advance(self, moment: float) -> None:
        while self._data.time < moment:
            mujoco.mj_step(self._model, self._data)
