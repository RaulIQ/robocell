"""Timed measurements from a rotating scanner."""

from typing import Protocol


class Rotation(Protocol):
    """Rotation that acquires only measurements due since the last call."""

    def advance(
        self, moment: float, start: float, count: int
    ) -> tuple[int, bytes]:
        """Return the new measurement count and encoded acquisitions."""
        ...
