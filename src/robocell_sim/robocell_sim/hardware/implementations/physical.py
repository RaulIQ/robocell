"""Step of MuJoCo physics alone."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Physical(Step):
    """Step that moves the MuJoCo simulation by one timestep and nothing more.

    The model fixes the length of the step, and the data holds the live
    state of the simulation.
    """

    _model: mujoco.MjModel
    _data: mujoco.MjData

    @override
    def take(self) -> None:
        mujoco.mj_step(self._model, self._data)
