"""Travel that has read the wheel encoders once."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.odometry.rolling import Rolling
from robocell_sim.robot.interfaces.kinematics import Kinematics
from robocell_sim.robot.interfaces.travel import Travel


@final
@dataclass(frozen=True)
class Started(Travel):
    """Travel that knows where the counts start but not yet how fast they go.

    The kinematics turn wheel angles into a distance and a turn, the
    resolution is the number of encoder ticks in one wheel revolution, the
    pose holds x and y in meters and the heading in radians, and the
    reading holds the left and right counts and the moment in seconds of
    the first reading. One reading spans no interval, so the robot counts
    as standing still until the next one.
    """

    _kinematics: Kinematics
    _resolution: int
    _pose: tuple[float, float, float]
    _reading: tuple[int, int, float]

    @override
    def read(self, left: int, right: int, moment: float) -> Travel:
        return Rolling(
            self._kinematics,
            self._resolution,
            self._pose,
            (self._reading, self._reading),
        ).read(left, right, moment)

    @override
    def position(self) -> tuple[float, float]:
        return (self._pose[0], self._pose[1])

    @override
    def heading(self) -> float:
        return self._pose[2]

    @override
    def velocity(self) -> tuple[float, float]:
        return (0.0, 0.0)
