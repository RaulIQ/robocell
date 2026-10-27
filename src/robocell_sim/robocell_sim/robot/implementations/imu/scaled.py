"""Reading scaled from the raw counts of a complete pair."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.pairing import Pairing
from robocell_sim.robot.interfaces.reading import Reading

UNIT = 0.001


@final
@dataclass(frozen=True)
class Scaled(Reading):
    """Reading that turns the raw counts of a pair into SI units.

    The pairing must be complete; each count is a thousandth of a rad/s
    for the gyro and of a m/s² for the accelerometer, as the protocol says.
    """

    _pairing: Pairing

    @override
    def rate(self) -> tuple[float, float, float]:
        x, y, z = self._pairing.measurement()[0]
        return (x * UNIT, y * UNIT, z * UNIT)

    @override
    def acceleration(self) -> tuple[float, float, float]:
        x, y, z = self._pairing.measurement()[1]
        return (x * UNIT, y * UNIT, z * UNIT)
