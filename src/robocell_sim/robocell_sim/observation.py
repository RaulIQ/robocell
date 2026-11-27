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
