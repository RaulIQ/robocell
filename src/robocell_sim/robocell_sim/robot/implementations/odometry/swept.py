"""Turn that adds up yaw rates over time."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.odometry import Odometry
from robocell_sim.robot.interfaces.turn import Turn


@final
@dataclass(frozen=True)
class Swept(Turn):
    """Turn that adds each yaw rate by the trapezoid rule.

    The angle is the turn in radians so far, and the rate and the stamp
    are those of the last reading, in rad/s and seconds. The next reading
    adds the mean of both rates times the time between their stamps, so
    the stamps may jitter without the sum drifting.
    """

    _angle: float
    _rate: float
    _stamp: float

    @override
    def read(self, rate: float, stamp: float) -> Turn:
        return Swept(
            self._angle + (self._rate + rate) / 2 * (stamp - self._stamp),
            rate,
            stamp,
        )

    @override
    def apply(self, odometry: Odometry, left: int, right: int) -> Odometry:
        return odometry.read(left, right, self._angle)
