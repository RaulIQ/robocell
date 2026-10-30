"""Odometry errors aligned by acquisition time and a fixed planar frame."""

import cmath
import math
from bisect import bisect_left
from dataclasses import dataclass
from typing import final, override

from docs.accuracy.comparison import Comparison


@final
@dataclass(frozen=True)
class Aligned(Comparison):
    """Comparison using interpolation within recorded truth, without fitting.

    Each sample holds time, x, y, yaw, forward speed and yaw rate. Reference
    poses are in world; estimate poses are in odom. Offset is the recorded
    world-to-odom x, y and yaw. Both twists are already in robot axes.
    """

    _reference: tuple[tuple[float, ...], ...]
    _estimate: tuple[tuple[float, ...], ...]
    _offset: tuple[float, float, float]

    @override
    def rows(self) -> tuple[tuple[float, ...], ...]:
        if not self._reference:
            message = "Ground truth has no samples for odometry comparison"
            raise ValueError(message)
        return tuple(
            self.errors(estimate)
            for estimate in self._estimate
            if self._reference[0][0] <= estimate[0] <= self._reference[-1][0]
        )

    @override
    def reference(self, moment: float) -> tuple[float, ...]:
        index = bisect_left(
            tuple(sample[0] for sample in self._reference), moment
        )
        if index == 0:
            return self._reference[0]
        earlier = self._reference[index - 1]
        later = self._reference[index]
        fraction = (moment - earlier[0]) / (later[0] - earlier[0])
        return tuple(
            earlier[column]
            + fraction
            * (
                math.remainder(later[column] - earlier[column], math.tau)
                if column == 3
                else later[column] - earlier[column]
            )
            for column in range(len(earlier))
        )

    @override
    def errors(self, estimate: tuple[float, ...]) -> tuple[float, ...]:
        reference = self.reference(estimate[0])
        position = complex(*estimate[1:3]) * cmath.rect(1, self._offset[2])
        return (
            estimate[0],
            abs(
                position + complex(*self._offset[:2]) - complex(*reference[1:3])
            ),
            math.remainder(
                estimate[3] + self._offset[2] - reference[3], math.tau
            ),
            estimate[4] - reference[4],
            estimate[5] - reference[5],
        )
