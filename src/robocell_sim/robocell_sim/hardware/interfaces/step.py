"""Step of the simulated world."""

from typing import Protocol


class Step(Protocol):
    """One step of the simulated world."""

    def take(self) -> None:
        """Move the world forward by one physics step."""
