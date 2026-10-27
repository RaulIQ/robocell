"""Measurement packed into the CAN frames of docs/protocols/imu.md."""

import struct
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.measurement import Measurement

GYRO = 0x101
ACCELEROMETER = 0x102
UNIT = 0.001
LAYOUT = struct.Struct("<hhhBx")


@final
@dataclass(frozen=True)
class Packed(Measurement):
    """Measurement that packs the gyro and the accelerometer into two frames.

    The rate holds the angular velocity in rad/s and the acceleration the
    linear one in m/s², both x, y and z in the imu_link frame, and the
    number tells this measurement from its neighbours. Each value goes out
    as an int16 count of thousandths, rounded to the nearest one and clipped
    to the range of int16, followed by the number and a reserved zero byte.
    """

    _rate: tuple[float, float, float]
    _acceleration: tuple[float, float, float]
    _number: int

    @override
    def frames(self) -> tuple[tuple[int, bytes], ...]:
        return tuple(
            (
                identifier,
                LAYOUT.pack(
                    *(
                        max(-32768, min(32767, round(value / UNIT)))
                        for value in vector
                    ),
                    self._number,
                ),
            )
            for identifier, vector in (
                (GYRO, self._rate),
                (ACCELEROMETER, self._acceleration),
            )
        )
