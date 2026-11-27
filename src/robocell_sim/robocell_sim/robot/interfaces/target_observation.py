"""The reading shared by visual target detectors."""

from typing import Protocol

Reading = tuple[tuple[float, float, float], str, float, bool]


class TargetObservation(Protocol):
    """A position measured in a coordinate frame at acquisition time."""

    def reading(self) -> Reading:
        """Report the measured position, frame, timestamp and validity."""
        ...
