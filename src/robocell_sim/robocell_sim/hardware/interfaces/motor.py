"""Immutable control state of one velocity drive."""

from __future__ import annotations

from typing import Protocol


class Motor(Protocol):
    """Motor control that returns a new state for each device event."""

    def command(
        self, word: int, target: int, moment: float, *, ready: bool
    ) -> Motor:
        """Accept one atomic controlword and velocity command."""
        ...

    def event(self, code: int, moment: float) -> Motor:
        """Record heartbeat (zero), inhibition (-1), or a fault code."""
        ...

    def advance(
        self,
        moment: float,
        velocity: float,
        watchdog: float,
        timeout: float,
    ) -> Motor:
        """Evaluate protective deadlines against monotonic time."""
        ...

    def reading(self) -> tuple[int, int, int]:
        """Return statusword, target and first fault code."""
        ...
