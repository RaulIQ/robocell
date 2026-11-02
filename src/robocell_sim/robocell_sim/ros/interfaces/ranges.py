"""Uniform ranges used by the existing collision geometry."""

from typing import Protocol


class Ranges(Protocol):
    """Nearest valid echo at each degree counterclockwise from ahead."""

    def ranges(self) -> tuple[float, ...]:
        """Return one range per degree with infinity for absent returns."""
        ...
