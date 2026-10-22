"""Kinematics of a differential drive."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.interfaces.kinematics import Kinematics


@final
@dataclass(frozen=True)
class Differential(Kinematics):
    """Differential drive with two coaxial wheels on a common axle.

    The radius is the wheel radius and the track is the distance between
    the wheel contact points, both in meters.
    """

    _radius: float
    _track: float

    @override
    def wheels(self, linear: float, angular: float) -> tuple[float, float]:
        return (
            (linear - angular * self._track / 2) / self._radius,
            (linear + angular * self._track / 2) / self._radius,
        )

    @override
    def velocity(self, left: float, right: float) -> tuple[float, float]:
        return (
            self._radius * (left + right) / 2,
            self._radius * (right - left) / self._track,
        )
