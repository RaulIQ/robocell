"""Velocity command from the arrow keys."""

from dataclasses import dataclass
from typing import final, override

import glfw

from robocell_sim.robot.interfaces.command import Command
from robocell_sim.robot.interfaces.speed import Speed


@final
@dataclass(frozen=True)
class Arrows(Command):
    """Body velocity steered with the arrow keys and stopped with enter.

    The up and down arrows step the linear speed, the left and right arrows
    step the counterclockwise yaw rate, and every other key changes nothing.
    """

    _linear: Speed
    _angular: Speed

    @override
    def press(self, key: int) -> Command:
        return {
            glfw.KEY_UP: Arrows(self._linear.up(), self._angular),
            glfw.KEY_DOWN: Arrows(self._linear.down(), self._angular),
            glfw.KEY_LEFT: Arrows(self._linear, self._angular.up()),
            glfw.KEY_RIGHT: Arrows(self._linear, self._angular.down()),
            glfw.KEY_ENTER: Arrows(self._linear.stop(), self._angular.stop()),
        }.get(key, self)

    @override
    def velocity(self) -> tuple[float, float]:
        return (self._linear.value(), self._angular.value())
