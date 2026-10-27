"""Speed that changes in equal steps."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.speed import Speed


@final
@dataclass(frozen=True)
class Stepped(Speed):
    """Speed made of a whole number of equal steps within a symmetric limit.

    The step is the size of one step, the top is the most steps allowed in
    either direction, and the level is the current number of steps.
    """

    _step: float
    _top: int
    _level: int

    @override
    def up(self) -> Speed:
        return Stepped(self._step, self._top, min(self._level + 1, self._top))

    @override
    def down(self) -> Speed:
        return Stepped(self._step, self._top, max(self._level - 1, -self._top))

    @override
    def stop(self) -> Speed:
        return Stepped(self._step, self._top, 0)

    @override
    def value(self) -> float:
        return self._step * self._level
