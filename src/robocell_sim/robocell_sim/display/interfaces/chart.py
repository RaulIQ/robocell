"""Chart that grows while the robot drives."""

from typing import Protocol


class Chart(Protocol):
    """Chart that takes in the positions that arrived since it last drew."""

    def draw(self) -> None:
        """Add the newly arrived positions to the chart."""
        ...
