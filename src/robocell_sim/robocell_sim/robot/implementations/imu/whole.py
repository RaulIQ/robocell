"""Pairing that holds both frames of one measurement."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.imu import waiting
from robocell_sim.robot.interfaces.frame import Frame
from robocell_sim.robot.interfaces.journal import Journal
from robocell_sim.robot.interfaces.pairing import Pairing


@final
@dataclass(frozen=True)
class Whole(Pairing):
    """Pairing that holds the gyro and the accelerometer frame of a measurement.

    Both frames carry the same number. The accelerometer frame that led
    here completes the pair and so loses nothing, and the next frame starts
    the next measurement, as after any other that ended.
    """

    _gyro: Frame
    _accelerometer: Frame

    @override
    def read(self, frame: Frame) -> Pairing:
        return waiting.Waiting(self._gyro.number(), 0).read(frame)

    @override
    def complete(self) -> bool:
        return True

    @override
    def measurement(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        return (self._gyro.values(), self._accelerometer.values())

    @override
    def note(self, journal: Journal) -> None:
        journal.lose(0)
