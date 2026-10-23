"""Physics that step up to a moment in time."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.interfaces.physics import Physics
from robocell_sim.interfaces.step import Step


@final
@dataclass(frozen=True)
class Paced(Physics):
    """Physics that take as many steps as a moment in time needs.

    The step moves the world forward by one timestep, and the data holds
    the live state of the simulation, whose clock never runs more than one
    step past the moment it was asked to reach.
    """

    _step: Step
    _data: mujoco.MjData

    @override
    def advance(self, moment: float) -> None:
        while self._data.time < moment:
            self._step.take()
