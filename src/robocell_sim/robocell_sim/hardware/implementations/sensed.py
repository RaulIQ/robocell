"""IMU that reads the MuJoCo sensors."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.imu import Imu


@final
@dataclass(frozen=True)
class Sensed(Imu):
    """IMU that reads a gyro and an accelerometer sensor of the model.

    The data holds the live state of the simulation, and the gyro and the
    accelerometer name the sensors on one site, so both measure in the
    frame of that site, imu_link on the Waffle Pi.
    """

    _data: mujoco.MjData
    _gyro: str
    _accelerometer: str

    @override
    def rate(self) -> tuple[float, float, float]:
        x, y, z = self._data.sensor(self._gyro).data
        return (float(x), float(y), float(z))

    @override
    def acceleration(self) -> tuple[float, float, float]:
        x, y, z = self._data.sensor(self._accelerometer).data
        return (float(x), float(y), float(z))
