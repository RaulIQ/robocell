"""Assembly of the synthetic wheel drive's three feedback PDOs."""

from typing import Protocol

Snapshot = tuple[int, int, int, int, int]


class Feedback(Protocol):
    """One coherent status, position, velocity, sample and acquisition time."""

    def read(self, part: int, payload: bytes, moment: float) -> "Feedback":
        """Consume one PDO fragment at a monotonic reception time."""
        ...

    def complete(self) -> bool:
        """Whether a complete coherent snapshot is available."""
        ...

    def measurement(self) -> Snapshot:
        """Status, counts, milliradians per second, sample and nanoseconds."""
        ...
