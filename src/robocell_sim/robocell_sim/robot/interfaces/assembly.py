"""A rotation under construction."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.sweep import Sweep


class Assembly(Protocol):
    """Assembly that discards damaged and incomplete startup turns."""

    def accept(
        self,
        echo: tuple[bool, bool, float, float, int],
        stamp: int,
        ceiling: int,
    ) -> tuple[Assembly, tuple[Sweep, ...]]:
        """Consume a ray with its estimated time and current ROS nanoseconds."""
        ...
