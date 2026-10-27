"""Pairing between two measurements of the IMU board."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.imu import half
from robocell_sim.robot.interfaces.frame import Frame
from robocell_sim.robot.interfaces.journal import Journal
from robocell_sim.robot.interfaces.pairing import Pairing


@final
@dataclass(frozen=True)
class Waiting(Pairing):
    """Pairing that waits for the gyro frame after the measurement last.

    The last number is that of the measurement that ended, paired or lost,
    and the count holds the measurements the frame that led here lost. A
    frame numbered further on loses the measurements skipped in between,
    and an accelerometer frame with no gyro frame before it loses its own.
    """

    _last: int
    _lost: int

    @override
    def read(self, frame: Frame) -> Pairing:
        if frame.gyro():
            return half.Half(frame, frame.gap(self._last))
        return Waiting(frame.number(), frame.gap(self._last) + 1)

    @override
    def complete(self) -> bool:
        return False

    @override
    def measurement(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        message = f"Measurement after {self._last} is not complete yet"
        raise ValueError(message)

    @override
    def note(self, journal: Journal) -> None:
        journal.lose(self._lost)
