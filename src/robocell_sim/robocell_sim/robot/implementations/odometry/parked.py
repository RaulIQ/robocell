"""Travel that has read no wheel encoder yet."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.odometry.started import Started
from robocell_sim.robot.interfaces.kinematics import Kinematics
from robocell_sim.robot.interfaces.travel import Travel


@final
@dataclass(frozen=True)
class Parked(Travel):
    """Travel that stands where the robot starts until the encoders speak.

    The kinematics and the resolution pass on to the travel that follows,
    and the pose holds x and y in meters and the heading in radians where
    the robot starts. The first reading only sets the counts and the moment
    to count from, so a driver that starts with the wheels already turned
    does not leap across the room.
    """

    _kinematics: Kinematics
    _resolution: int
    _pose: tuple[float, float, float]

    @override
    def read(self, left: int, right: int, moment: float) -> Travel:
        return Started(
            self._kinematics,
            self._resolution,
            self._pose,
            (left, right, moment),
        )

    @override
    def position(self) -> tuple[float, float]:
        return (self._pose[0], self._pose[1])

    @override
    def heading(self) -> float:
        return self._pose[2]

    @override
    def velocity(self) -> tuple[float, float]:
        return (0.0, 0.0)
