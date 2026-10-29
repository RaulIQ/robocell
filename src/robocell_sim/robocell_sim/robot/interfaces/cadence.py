"""A sensor's declared sampling cadence."""

from typing import Protocol


class Cadence(Protocol):
    """Cadence validated against the sensor's binary response."""

    def period(self) -> int:
        """Return a positive standard sample period in microseconds."""
        ...
