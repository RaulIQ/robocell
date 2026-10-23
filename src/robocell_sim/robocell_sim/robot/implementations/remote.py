"""Velocity command that a ROS 2 operator steers."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.command import Command


@final
@dataclass(frozen=True)
class Remote(Command):
    """Body velocity set directly over ROS 2, ahead of the arrow keys.

    The origin is the command of the arrow keys that the operator left
    behind, which a key press resumes, while the linear speed in m/s and
    the counterclockwise yaw rate in rad/s come from the latest steer.
    """

    _origin: Command
    _linear: float
    _angular: float

    @override
    def press(self, key: int) -> Command:
        return self._origin.press(key)

    @override
    def steer(self, linear: float, angular: float) -> Command:
        return Remote(self._origin, linear, angular)

    @override
    def velocity(self) -> tuple[float, float]:
        return (self._linear, self._angular)
