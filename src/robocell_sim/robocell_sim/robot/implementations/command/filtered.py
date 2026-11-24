"""One command path with application input timestamps."""

from dataclasses import dataclass, replace
from typing import final, override

from robocell_sim.robot.interfaces.command import Command
from robocell_sim.robot.interfaces.control import Control
from robocell_sim.robot.interfaces.pilot import Command as Request


@final
@dataclass(frozen=True)
class Filtered(Control):
    """Control that retains the latest body command and its source time."""

    _origin: Command
    _moment: float

    @override
    def steer(self, velocity: tuple[float, float], moment: float) -> Control:
        return (
            replace(self, _origin=self._origin.steer(*velocity), _moment=moment)
            if moment > self._moment
            else self
        )

    @override
    def reading(self) -> Request:
        return (*self._origin.velocity(), self._moment)
