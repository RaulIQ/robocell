"""Turn before the first yaw rate."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.odometry.swept import Swept
from robocell_sim.robot.interfaces.odometry import Odometry
from robocell_sim.robot.interfaces.turn import Turn


@final
@dataclass(frozen=True)
class Still(Turn):
    """Turn that has heard no yaw rate yet, so it knows no time to count from.

    The angle is the turn in radians the robot starts with. The first rate
    adds nothing, since no time has passed before it, and only starts the
    clock that the next rates count from.
    """

    _angle: float

    @override
    def read(self, rate: float, stamp: float) -> Turn:
        return Swept(self._angle, rate, stamp)

    @override
    def apply(self, odometry: Odometry, left: int, right: int) -> Odometry:
        return odometry.read(left, right, self._angle)
