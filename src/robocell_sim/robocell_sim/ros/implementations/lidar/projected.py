"""One acquired echo expressed at the scan reference pose."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.ros.interfaces.coordinates import Coordinates
from robocell_sim.ros.interfaces.echo import Echo


@final
@dataclass(frozen=True)
class Projected(Echo):
    """Echo whose range and bearing account for sensor motion."""

    _sample: tuple[float, float, float]
    _coordinates: Coordinates
    _interval: tuple[int, int]
    _limits: tuple[float, float]

    @override
    def reading(self) -> tuple[float, float, float]:
        point = self._coordinates.point(
            (
                self._sample[0] * math.cos(self._sample[1]),
                self._sample[0] * math.sin(self._sample[1]),
            ),
            *self._interval,
        )
        distance = math.hypot(*point)
        return (
            distance
            if self._limits[0] <= distance <= self._limits[1]
            else math.inf,
            math.atan2(point[1], point[0]),
            self._sample[2],
        )
