"""Command that holds the robot back from lidar echoes."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.command import Command


@final
@dataclass(frozen=True)
class Guarded(Command):
    """Command that refuses to drive into what the lidar sees.

    The origin is the command of the operator, the scan holds the latest
    ranges in meters, one per degree counterclockwise from ahead, the reach
    is how far ahead and behind the scanner a corridor stretches, and the
    width is how far it spreads to each side, both in meters. An echo in
    the corridor ahead stops driving forward, one in the corridor behind
    stops reversing, and turning in place stays free to steer away.
    """

    _origin: Command
    _scan: tuple[float, ...]
    _reach: float
    _width: float

    @override
    def press(self, key: int) -> Command:
        return Guarded(
            self._origin.press(key), self._scan, self._reach, self._width
        )

    @override
    def velocity(self) -> tuple[float, float]:
        linear, angular = self._origin.velocity()
        echoes = [
            distance * math.cos(math.radians(index))
            for index, distance in enumerate(self._scan)
            if math.isfinite(distance)
            and abs(distance * math.sin(math.radians(index))) <= self._width
        ]
        return (
            max(
                min(
                    linear,
                    0.0
                    if any(0 < echo <= self._reach for echo in echoes)
                    else math.inf,
                ),
                0.0
                if any(-self._reach <= echo < 0 for echo in echoes)
                else -math.inf,
            ),
            angular,
        )
