"""The common observation supplied by visual target detectors."""

from dataclasses import dataclass
from typing import final


@final
@dataclass(frozen=True)
class Observation:
    """A target position in a named frame at its acquisition time."""

    _position: tuple[float, float, float]
    _frame: str
    _stamp: float
    _valid: bool

    def position(self) -> tuple[float, float, float]:
        return self._position

    def frame(self) -> str:
        return self._frame

    def stamp(self) -> float:
        return self._stamp

    def valid(self) -> bool:
        return self._valid
