"""Inaccuracy of a MEMS sensor with a fixed bias and a normal noise."""

from dataclasses import dataclass
from typing import final, override

import numpy as np

from robocell_sim.hardware.interfaces.inaccuracy import Inaccuracy


@final
@dataclass(frozen=True)
class Gaussian(Inaccuracy):
    """Inaccuracy that shifts each axis by a bias and adds a normal noise.

    The bias holds the constant offset of x, y and z in the units of the
    sensor, the noise is the standard deviation of the normal error each
    axis gets anew on every reading, and the generator draws that error.
    Every reading draws exactly three numbers, so the readings depend only
    on the generator and on how many came before.
    """

    _bias: tuple[float, float, float]
    _noise: float
    _generator: np.random.Generator

    @override
    def reading(
        self, value: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        x, y, z = (
            np.array(value)
            + np.array(self._bias)
            + self._generator.normal(0.0, self._noise, 3)
        )
        return (float(x), float(y), float(z))
