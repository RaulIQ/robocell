"""Wheel read from a MuJoCo encoder sensor."""

import math
from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.wheel import Wheel


@final
@dataclass(frozen=True)
class Simulated(Wheel):
    """Wheel that counts the angle of a MuJoCo joint position sensor.

    The data holds the live state of the simulation, the sensor names the
    jointpos sensor that serves as the wheel encoder, and the resolution is
    the number of ticks in one revolution of the wheel.
    """

    _data: mujoco.MjData
    _sensor: str
    _resolution: int

    @override
    def ticks(self) -> int:
        return math.floor(
            float(self._data.sensor(self._sensor).data[0])
            * self._resolution
            / (2 * math.pi)
        )

    @override
    def angle(self) -> float:
        return self.ticks() * 2 * math.pi / self._resolution
