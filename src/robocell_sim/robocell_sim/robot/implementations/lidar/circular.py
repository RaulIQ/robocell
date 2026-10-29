"""A complete chronological rotation ready for ROS publication."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.lidar.binned import Binned
from robocell_sim.robot.interfaces.sweep import Sweep


@final
@dataclass(frozen=True)
class Circular(Sweep):
    """Sweep with decreasing ROS angles and increasing acquisition time."""

    _rays: tuple[tuple[float, float, int], ...]
    _stamp: int
    _duration: float
    _limits: tuple[float, float]

    @override
    def ranges(self) -> tuple[float, ...]:
        bins = Binned(self._rays, self._limits).returns()
        return tuple(
            bins.get(index, (math.inf, 0))[0]
            for index in range(len(self._rays))
        )

    @override
    def intensities(self) -> tuple[float, ...]:
        bins = Binned(self._rays, self._limits).returns()
        return tuple(
            float(bins.get(index, (math.inf, 0))[1])
            for index in range(len(self._rays))
        )

    @override
    def geometry(self) -> tuple[float, float, float]:
        first = -math.radians(self._rays[0][0])
        increment = -2 * math.pi / len(self._rays)
        return (first, first + (len(self._rays) - 1) * increment, increment)

    @override
    def timing(self) -> tuple[int, float, float]:
        return (self._stamp, self._duration, self._duration / len(self._rays))
