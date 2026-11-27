"""State transitions for safe target following."""

from dataclasses import dataclass, replace
from typing import final


@final
@dataclass(frozen=True)
class Pursuit:
    """Coordinate operator intent, goal lifetime and target availability."""

    _requested: bool
    _active: bool
    _starting: bool
    _available: bool

    def toggle(self) -> "Pursuit":
        return replace(self, _requested=not self._requested)

    def observe(self) -> "Pursuit":
        return replace(self, _available=True)

    def lose(self) -> "Pursuit":
        return replace(self, _available=False)

    def begin(self) -> "Pursuit":
        return replace(self, _starting=True)

    def accept(self) -> "Pursuit":
        return replace(self, _active=True, _starting=False)

    def finish(self) -> "Pursuit":
        return replace(self, _active=False, _starting=False)

    def starts(self) -> bool:
        return (
            self._requested
            and self._available
            and not self._active
            and not self._starting
        )

    def cancels(self) -> bool:
        return self._active and (not self._requested or not self._available)

    def requested(self) -> bool:
        return self._requested
