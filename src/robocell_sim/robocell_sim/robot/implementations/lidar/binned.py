"""Received angles placed on one uniform clockwise grid."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.bins import Bins


@final
@dataclass(frozen=True)
class Binned(Bins):
    """Bins anchored at the first acquired ray, with one slot per sample.

    The grid follows clockwise acquisition order rather than reversing time.
    Packet angle quantization is absorbed by nearest angular binning.
    """

    _rays: tuple[tuple[float, float, int], ...]
    _limits: tuple[float, float]

    @override
    def returns(self) -> dict[int, tuple[float, int]]:
        return {
            min(
                len(self._rays) - 1,
                round(
                    ((angle - self._rays[0][0]) % 360) * len(self._rays) / 360
                ),
            ): (
                distance
                if self._limits[0] <= distance <= self._limits[1] and quality
                else math.inf,
                quality
                if self._limits[0] <= distance <= self._limits[1]
                else 0,
            )
            for angle, distance, quality in self._rays
        }
