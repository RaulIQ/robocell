"""Commit of complete coherent wheel feedback."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.feedback import Feedback
from robocell_sim.robot.interfaces.position import Position
from robocell_sim.robot.interfaces.recording import Acquisition, Recording


@final
@dataclass(frozen=True)
class Recorded(Recording):
    """Acquisition that rejects backwards epochs and ignores duplicate time."""

    _assembly: Feedback
    _position: Position
    _readings: tuple[Acquisition, ...]

    @override
    def commit(self, moment: float) -> tuple[Position, tuple[Acquisition, ...]]:
        sample = self._assembly.measurement()
        if self._readings and sample[4] < self._readings[0][0][4]:
            error = (
                "Wheel acquisition clock moved backwards "
                "and invalidated its epoch"
            )
            raise ValueError(error)
        return (
            (self._position, self._readings)
            if self._readings and sample[4] == self._readings[0][0][4]
            else (
                self._position.read(sample[1], sample[4]),
                ((sample, moment),),
            )
        )
