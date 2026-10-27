"""Gyro angle kept in the simulation user data."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.gyro import Gyro


@final
@dataclass(frozen=True)
class Accumulated(Gyro):
    """Gyro that reads the angle the integrating step adds up in user data.

    The data holds the live state of the simulation, and the slot is the
    user data entry that keeps the angle.
    """

    _data: mujoco.MjData
    _slot: int

    @override
    def angle(self) -> float:
        return float(self._data.userdata[self._slot])
