"""Pairing of the frames of the IMU board."""

from typing import Protocol

from robocell_sim.robot.interfaces.frame import Frame
from robocell_sim.robot.interfaces.journal import Journal


class Pairing(Protocol):
    """Pairing of the gyro and the accelerometer frame of one measurement."""

    def read(self, frame: Frame) -> "Pairing":
        """Return the pairing after one more frame came off the bus."""
        ...

    def complete(self) -> bool:
        """Tell whether both frames of one measurement are in."""
        ...

    def measurement(self) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
        """Return the raw gyro and accelerometer values of the pair."""
        ...

    def note(self, journal: Journal) -> None:
        """Tell the journal how many measurements the last frame lost."""
        ...
