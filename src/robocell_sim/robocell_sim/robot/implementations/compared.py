"""Slip that compares the encoder turn with the gyro turn."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.kinematics import Kinematics
from robocell_sim.robot.interfaces.slip import Slip


@final
@dataclass(frozen=True)
class Compared(Slip):
    """Slip that flags turn rates of the encoders and the gyro drifting apart.

    The kinematics turn wheel angles into a turn, the resolution is the
    number of encoder ticks in one wheel revolution, the threshold is the
    largest gap in turn rate in rad/s that still counts as grip, and the
    readings hold the left and right counts, the gyro angle and the moment
    of the last two readings. Both rates are averaged over the same
    interval, so comparing the turns against the threshold times the
    interval needs no division.
    """

    _kinematics: Kinematics
    _resolution: int
    _threshold: float
    _readings: tuple[
        tuple[int, int, float, float], tuple[int, int, float, float]
    ]

    @override
    def read(self, left: int, right: int, angle: float, moment: float) -> Slip:
        return Compared(
            self._kinematics,
            self._resolution,
            self._threshold,
            (self._readings[1], (left, right, angle, moment)),
        )

    @override
    def slipping(self) -> bool:
        earlier, later = self._readings
        return abs(
            self._kinematics.velocity(
                (later[0] - earlier[0]) * 2 * math.pi / self._resolution,
                (later[1] - earlier[1]) * 2 * math.pi / self._resolution,
            )[1]
            - (later[2] - earlier[2])
        ) > self._threshold * (later[3] - earlier[3])
