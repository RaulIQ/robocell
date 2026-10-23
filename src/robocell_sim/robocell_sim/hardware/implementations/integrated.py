"""Step that integrates the gyro into a heading."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Integrated(Step):
    """Step that adds the turn the gyro measured to the angle in user data.

    The origin moves the world, the data holds the live state of the
    simulation, the sensor names the gyro, and the slot is the user data
    entry that keeps the angle. MuJoCo measures the gyro at the start of a
    step, so its yaw rate times the time the step took is the turn over it.
    """

    _origin: Step
    _data: mujoco.MjData
    _sensor: str
    _slot: int

    @override
    def take(self) -> None:
        moment = self._data.time
        self._origin.take()
        self._data.userdata[self._slot] += float(
            self._data.sensor(self._sensor).data[2]
        ) * (self._data.time - moment)
