"""Kinematics that keep the wheels within their speed limit."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.interfaces.kinematics import Kinematics


@final
@dataclass(frozen=True)
class Limited(Kinematics):
    """Kinematics that scale both wheels down together past a speed limit.

    The origin turns the body velocity into wheel speeds, and the top is the
    fastest a wheel may spin in rad/s. Scaling both wheels by the same factor
    keeps their ratio, so the robot drives the commanded arc, only slower.
    """

    _origin: Kinematics
    _top: float

    @override
    def wheels(self, linear: float, angular: float) -> tuple[float, float]:
        wheels = self._origin.wheels(linear, angular)
        factor = self._top / max(abs(wheels[0]), abs(wheels[1]), self._top)
        return (wheels[0] * factor, wheels[1] * factor)

    @override
    def velocity(self, left: float, right: float) -> tuple[float, float]:
        return self._origin.velocity(left, right)
