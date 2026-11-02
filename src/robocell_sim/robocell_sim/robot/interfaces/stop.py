"""Immediate inhibition of reachable wheel drives."""

from typing import Protocol


class Stop(Protocol):
    """Best-effort stop that still reaches the second drive after a failure."""

    def apply(self, word: int) -> None:
        """Send a zero target with a chosen protective controlword."""
        ...
