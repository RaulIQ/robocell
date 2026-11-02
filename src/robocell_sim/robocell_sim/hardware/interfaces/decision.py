"""Atomic result of one drive controlword."""

from typing import Protocol

Outcome = tuple[tuple[int, int, int], tuple[int, float, float, float]]


class Decision(Protocol):
    """Decision that returns control state and its accepted history."""

    def result(self) -> Outcome:
        """Return status, output, fault and control history."""
        ...
