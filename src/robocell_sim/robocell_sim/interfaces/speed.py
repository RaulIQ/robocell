"""Speed along one axis of motion."""

from typing import Protocol


class Speed(Protocol):
    """Speed along one axis of motion that changes in discrete steps."""

    def up(self) -> "Speed":
        """Speed one step higher."""

    def down(self) -> "Speed":
        """Speed one step lower."""

    def stop(self) -> "Speed":
        """Speed at rest."""

    def value(self) -> float:
        """Speed in its own units, such as m/s or rad/s."""
