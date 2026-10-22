"""Odometry reckoned from wheel encoder counts."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.interfaces.kinematics import Kinematics
from robocell_sim.interfaces.odometry import Odometry


@final
@dataclass(frozen=True)
class Reckoned(Odometry):
    """Pose that sums the moves the wheel encoders report, reading by reading.

    The kinematics turn wheel angles into a distance and a turn, the
    resolution is the number of encoder ticks in one wheel revolution, the
    pose holds x and y in meters and the heading in radians, and the ticks
    hold the left and right counts of the last reading.
    """

    _kinematics: Kinematics
    _resolution: int
    _pose: tuple[float, float, float]
    _ticks: tuple[int, int]

    @override
    def read(self, left: int, right: int) -> Odometry:
        distance, turn = self._kinematics.velocity(
            (left - self._ticks[0]) * 2 * math.pi / self._resolution,
            (right - self._ticks[1]) * 2 * math.pi / self._resolution,
        )
        heading = self._pose[2] + turn / 2
        return Reckoned(
            self._kinematics,
            self._resolution,
            (
                self._pose[0] + distance * math.cos(heading),
                self._pose[1] + distance * math.sin(heading),
                self._pose[2] + turn,
            ),
            (left, right),
        )

    @override
    def position(self) -> tuple[float, float]:
        return (self._pose[0], self._pose[1])
