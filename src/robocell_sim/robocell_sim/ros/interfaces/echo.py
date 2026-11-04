"""A projected return in a common reference frame."""

from typing import Protocol


class Echo(Protocol):
    """Distance, bearing and strength after motion compensation."""

    def reading(self) -> tuple[float, float, float]:
        """Return the projected polar observation."""
        ...
