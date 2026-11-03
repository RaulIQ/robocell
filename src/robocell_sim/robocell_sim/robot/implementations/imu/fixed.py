"""A gyro offset frozen after stationary calibration."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.imu import zeroed
from robocell_sim.robot.implementations.imu.averaged import Averaged
from robocell_sim.robot.interfaces.zero import Zero


@final
@dataclass(frozen=True)
class Fixed(Zero):
    """Established offset independent of subsequent robot motion.

    A backwards sensor clock starts a new calibration epoch because the
    simulated board may have restarted with a different constant offset.
    """

    _estimate: tuple[tuple[float, float, float], tuple[float, float, float]]
    _moment: float
    _limits: tuple[float, int, float]

    @override
    def read(
        self, rate: tuple[float, float, float], moment: float, *, still: bool
    ) -> Zero:
        if not all(math.isfinite(value) for value in (*rate, moment)):
            error = "Gyro calibration received a nonfinite measurement or time"
            raise ValueError(error)
        result: Zero = Fixed(self._estimate, moment, self._limits)
        if moment < self._moment:
            result = zeroed.Zeroed(
                self._limits, (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
            ).read(rate, moment, still=still)
        return result

    @override
    def ready(self) -> bool:
        return True

    @override
    def bias(self) -> tuple[float, float, float]:
        return self._estimate[0]

    @override
    def variance(self) -> tuple[float, float, float]:
        return self._estimate[1]
