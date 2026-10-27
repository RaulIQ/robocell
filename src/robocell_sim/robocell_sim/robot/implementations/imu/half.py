"""Pairing that holds the gyro frame of a measurement."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.imu import waiting, whole
from robocell_sim.robot.implementations.imu.received import NUMBERS
from robocell_sim.robot.interfaces.frame import Frame
from robocell_sim.robot.interfaces.journal import Journal
from robocell_sim.robot.interfaces.pairing import Pairing


@final
@dataclass(frozen=True)
class Half(Pairing):
    """Pairing that holds a gyro frame and waits for its accelerometer frame.

    The gyro is the frame that came first, and the count holds the
    measurements the frame that led here lost. An accelerometer frame with
    the same number completes the pair. Any other frame means this
    measurement will never complete, so the frame goes on as if the last
    measurement had been the one before, and the gap it finds counts this
    one among the lost.
    """

    _gyro: Frame
    _lost: int

    @override
    def read(self, frame: Frame) -> Pairing:
        if not frame.gyro() and frame.number() == self._gyro.number():
            return whole.Whole(self._gyro, frame)
        return waiting.Waiting((self._gyro.number() - 1) % NUMBERS, 0).read(
            frame
        )

    @override
    def complete(self) -> bool:
        return False

    @override
    def measurement(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        message = (
            f"Measurement {self._gyro.number()} is not complete "
            "without its accelerometer frame"
        )
        raise ValueError(message)

    @override
    def note(self, journal: Journal) -> None:
        journal.lose(self._lost)
