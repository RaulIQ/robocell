"""Odometry that turns by the gyro and moves by the wheel encoders."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.kinematics import Kinematics
from robocell_sim.robot.interfaces.odometry import Odometry


@final
@dataclass(frozen=True)
class Gyroscopic(Odometry):
    """Pose that moves by the encoders and turns by the gyro.

    The kinematics turn wheel angles into a distance, the resolution is the
    number of encoder ticks in one wheel revolution, the pose holds x and y
    in meters and the heading in radians, and the reading holds the left and
    right counts and the gyro angle of the last reading.
    """

    _kinematics: Kinematics
    _resolution: int
    _pose: tuple[float, float, float]
    _reading: tuple[int, int, float]

    @override
    def read(self, left: int, right: int, angle: float) -> Odometry:
        distance, _ = self._kinematics.velocity(
            (left - self._reading[0]) * 2 * math.pi / self._resolution,
            (right - self._reading[1]) * 2 * math.pi / self._resolution,
        )
        heading = self._pose[2] + (angle - self._reading[2]) / 2
        return Gyroscopic(
            self._kinematics,
            self._resolution,
            (
                self._pose[0] + distance * math.cos(heading),
                self._pose[1] + distance * math.sin(heading),
                self._pose[2] + angle - self._reading[2],
            ),
            (left, right, angle),
        )

    @override
    def position(self) -> tuple[float, float]:
        return (self._pose[0], self._pose[1])
