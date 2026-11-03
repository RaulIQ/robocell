"""Stationary estimation of the constant gyro offset."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.imu import fixed
from robocell_sim.robot.implementations.imu.averaged import Averaged
from robocell_sim.robot.interfaces.moments import Moments
from robocell_sim.robot.interfaces.zero import Zero

_GAP = 0.05
_MAXIMUM = 0.1
_MINIMUM = 2


@final
@dataclass(frozen=True)
class Zeroed(Zero):
    """Offset learned once from consecutive stationary measurements.

    Limits hold duration, minimum sample count and maximum deviation. The
    period holds first and latest timestamps, and moments hold the mean and
    the sum of squared residuals. Pauses supply no samples; gaps above 50 ms
    restart the window. A completed estimate becomes a fixed offset.
    """

    _limits: tuple[float, int, float]
    _period: tuple[float, float]
    _moments: Moments

    @override
    def read(
        self, rate: tuple[float, float, float], moment: float, *, still: bool
    ) -> Zero:
        if not all(math.isfinite(value) for value in (*rate, moment)):
            error = "Gyro calibration received a nonfinite measurement or time"
            raise ValueError(error)
        result = self
        if not still or max(abs(value) for value in rate) > _MAXIMUM:
            result = Zeroed(
                self._limits,
                (moment, moment),
                Averaged((0, (0.0,) * 3, (0.0,) * 3)),
            )
        elif (
            not self._moments.count()
            or not 0 <= moment - self._period[1] <= _GAP
            or (
                self._moments.count() >= self._limits[1]
                and self._period[1] - self._period[0] >= self._limits[0]
            )
            or any(
                abs(value - mean) > 4 * self._limits[2]
                for value, mean in zip(rate, self._moments.mean(), strict=True)
            )
        ):
            result = Zeroed(
                self._limits, (moment, moment), Averaged((1, rate, (0.0,) * 3))
            )
        elif moment > self._period[1]:
            result = Zeroed(
                self._limits,
                (self._period[0], moment),
                self._moments.read(rate),
            )
        return (
            fixed.Fixed(
                (result.bias(), result.variance()), moment, self._limits
            )
            if result.ready()
            else result
        )

    @override
    def ready(self) -> bool:
        return (
            self._moments.count() >= max(_MINIMUM, self._limits[1])
            and self._period[1] - self._period[0] >= self._limits[0]
            and max(self._moments.variance()) <= self._limits[2] ** 2
        )

    @override
    def bias(self) -> tuple[float, float, float]:
        if not self.ready():
            error = "Gyro offset is unavailable before stationary calibration"
            raise ValueError(error)
        return self._moments.mean()

    @override
    def variance(self) -> tuple[float, float, float]:
        if not self.ready():
            error = "Gyro uncertainty is unavailable before calibration"
            raise ValueError(error)
        return (
            self._moments.variance()[0] / self._moments.count(),
            self._moments.variance()[1] / self._moments.count(),
            self._moments.variance()[2] / self._moments.count(),
        )
