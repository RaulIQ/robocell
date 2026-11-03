"""Acquisition intervals of a sequential sensor."""

from __future__ import annotations

from typing import Protocol


class Chronology(Protocol):
    """Chronology that reserves nonoverlapping intervals in sensor time."""

    def read(
        self, first: int, duration: int, ceiling: int
    ) -> tuple[Chronology, tuple[int, ...]]:
        """Reserve nanoseconds before ROS time, or discard the turn."""
        ...

    def reset(self) -> Chronology:
        """Forget the arrival anchor while preserving published intervals."""
        ...
