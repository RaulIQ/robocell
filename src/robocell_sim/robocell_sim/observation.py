"""The common observation supplied by visual target detectors."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.target_observation import (
    Reading,
    TargetObservation,
)


@final
@dataclass(frozen=True)
class Observation(TargetObservation):
    """A measured target position and its acquisition metadata."""

    _position: tuple[float, float, float]
    _frame: str
    _stamp: float
    _valid: bool

    @override
    def reading(self) -> Reading:
        return self._position, self._frame, self._stamp, self._valid
