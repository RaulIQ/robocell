"""A rotation under construction."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.sweep import Sweep


class Assembly(Protocol):
    """Assembly that discards damaged and incomplete startup turns."""

    def accept(
        self, echo: tuple[bool, bool, float, float, int], stamp: int
    ) -> tuple[Assembly, tuple[Sweep, ...]]:
        """Consume a checked measurement and return completed turns."""
        ...
