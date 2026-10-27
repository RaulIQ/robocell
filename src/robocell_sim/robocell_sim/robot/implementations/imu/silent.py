"""Pairing before the first frame of the IMU board."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.imu import half, waiting
from robocell_sim.robot.interfaces.frame import Frame
from robocell_sim.robot.interfaces.journal import Journal
from robocell_sim.robot.interfaces.pairing import Pairing


@final
@dataclass(frozen=True)
class Silent(Pairing):
    """Pairing that has heard no frame yet, so it knows no number to follow.

    The count holds the measurements lost before the start, which is none
    for a driver that just started. The first frame sets the number to
    follow and skips nothing, whatever its number, so a driver that starts
    in the middle of the stream reports no false losses.
    """

    _lost: int

    @override
    def read(self, frame: Frame) -> Pairing:
        if frame.gyro():
            return half.Half(frame, 0)
        return waiting.Waiting(frame.number(), 1)

    @override
    def complete(self) -> bool:
        return False

    @override
    def measurement(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        message = "Measurement is not complete before the first frame"
        raise ValueError(message)

    @override
    def note(self, journal: Journal) -> None:
        journal.lose(self._lost)
