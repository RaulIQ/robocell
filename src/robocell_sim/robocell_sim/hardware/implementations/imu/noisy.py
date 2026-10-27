"""IMU whose sensors read with the inaccuracy of a real chip."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.imu import Imu
from robocell_sim.hardware.interfaces.inaccuracy import Inaccuracy


@final
@dataclass(frozen=True)
class Noisy(Imu):
    """IMU that passes each true measurement through its sensor inaccuracy.

    The origin measures the true turn and acceleration, the gyro holds the
    inaccuracy of the rate and the accelerometer that of the acceleration.
    Each call reads the inaccuracy once, so a board that asks once per
    measurement draws its noise once per measurement.
    """

    _origin: Imu
    _gyro: Inaccuracy
    _accelerometer: Inaccuracy

    @override
    def rate(self) -> tuple[float, float, float]:
        return self._gyro.reading(self._origin.rate())

    @override
    def acceleration(self) -> tuple[float, float, float]:
        return self._accelerometer.reading(self._origin.acceleration())
