"""Independent freshness of commands, heartbeat and acquisition."""

from typing import Protocol


class Supervision(Protocol):
    """Protective checks that precede command renewal."""

    def verify(self, moment: float, clock: int) -> None:
        """Verify host time and application-input freshness."""
        ...

    def feedback(self, moment: float, clock: int) -> None:
        """Verify each drive independently of the other drive."""
        ...
