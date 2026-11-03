"""Constant-memory statistics of three-axis gyro readings."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.moments import Moments

_MINIMUM = 2


@final
@dataclass(frozen=True)
class Averaged(Moments):
    """Welford statistics holding count, mean and squared residual sums."""

    _summary: tuple[int, tuple[float, float, float], tuple[float, float, float]]

    @override
    def read(self, value: tuple[float, float, float]) -> Moments:
        mean = tuple(
            mean + (sample - mean) / (self._summary[0] + 1)
            for sample, mean in zip(value, self._summary[1], strict=True)
        )
        scatter = tuple(
            scatter + (sample - previous) * (sample - current)
            for sample, previous, current, scatter in zip(
                value, self._summary[1], mean, self._summary[2], strict=True
            )
        )
        return Averaged(
            (
                self._summary[0] + 1,
                (mean[0], mean[1], mean[2]),
                (scatter[0], scatter[1], scatter[2]),
            )
        )

    @override
    def count(self) -> int:
        return self._summary[0]

    @override
    def mean(self) -> tuple[float, float, float]:
        return self._summary[1]

    @override
    def variance(self) -> tuple[float, float, float]:
        if self._summary[0] < _MINIMUM:
            error = "Gyro variance is unavailable before two measurements"
            raise ValueError(error)
        return (
            self._summary[2][0] / (self._summary[0] - 1),
            self._summary[2][1] / (self._summary[0] - 1),
            self._summary[2][2] / (self._summary[0] - 1),
        )
