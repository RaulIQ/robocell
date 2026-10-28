"""Travel that the wheel encoders reckon reading by reading."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.kinematics import Kinematics
from robocell_sim.robot.interfaces.travel import Travel


@final
@dataclass(frozen=True)
class Rolling(Travel):
    """Travel that sums the moves of the wheel encoders and times the last one.

    The kinematics turn wheel angles into a distance and a turn, the
    resolution is the number of encoder ticks in one wheel revolution, the
    pose holds x and y in meters and the heading in radians, and the
    readings hold the left and right counts and the moment in seconds of
    the last two readings. The heading turns by the encoders alone, and the
    velocity is the move between the two readings over the time between
    them, so two readings at the same moment cannot give a velocity.
    """

    _kinematics: Kinematics
    _resolution: int
    _pose: tuple[float, float, float]
    _readings: tuple[tuple[int, int, float], tuple[int, int, float]]

    @override
    def read(self, left: int, right: int, moment: float) -> Travel:
        distance, turn = self._kinematics.velocity(
            (left - self._readings[1][0]) * 2 * math.pi / self._resolution,
            (right - self._readings[1][1]) * 2 * math.pi / self._resolution,
        )
        heading = self._pose[2] + turn / 2
        return Rolling(
            self._kinematics,
            self._resolution,
            (
                self._pose[0] + distance * math.cos(heading),
                self._pose[1] + distance * math.sin(heading),
                self._pose[2] + turn,
            ),
            (self._readings[1], (left, right, moment)),
        )

    @override
    def position(self) -> tuple[float, float]:
        return (self._pose[0], self._pose[1])

    @override
    def heading(self) -> float:
        return self._pose[2]

    @override
    def velocity(self) -> tuple[float, float]:
        earlier, later = self._readings
        return self._kinematics.velocity(
            (later[0] - earlier[0])
            * 2
            * math.pi
            / self._resolution
            / (later[2] - earlier[2]),
            (later[1] - earlier[1])
            * 2
            * math.pi
            / self._resolution
            / (later[2] - earlier[2]),
        )
