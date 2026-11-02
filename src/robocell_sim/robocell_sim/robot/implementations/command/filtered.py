"""One command path with independent scan and application freshness."""

from dataclasses import dataclass, replace
from typing import final, override

from robocell_sim.robot.implementations.command.guarded import Guarded
from robocell_sim.robot.interfaces.command import Command
from robocell_sim.robot.interfaces.control import Control
from robocell_sim.robot.interfaces.pilot import Command as Request


@final
@dataclass(frozen=True)
class Filtered(Control):
    """Control that never turns a fresh scan into a fresh body command."""

    _origin: Command
    _moment: float
    _scan: tuple[tuple[float, ...], int]
    _limits: tuple[float, float, int]

    @override
    def steer(self, velocity: tuple[float, float], moment: float) -> Control:
        return (
            replace(self, _origin=self._origin.steer(*velocity), _moment=moment)
            if moment > self._moment
            else self
        )

    @override
    def scan(self, ranges: tuple[float, ...], stamp: int) -> Control:
        return replace(self, _scan=(ranges, stamp))

    @override
    def reading(self, clock: int, *, protected: bool) -> Request:
        velocity = self._origin.velocity()
        if protected:
            velocity = (
                Guarded(
                    self._origin, self._scan[0], *self._limits[:2]
                ).velocity()
                if self._scan[0]
                and self._scan[1] >= 0
                and 0 <= clock - self._scan[1] <= self._limits[2]
                else (0.0, 0.0)
            )
        return (*velocity, self._moment)
