"""Commit of one validated wheel acquisition."""

from typing import Protocol

from robocell_sim.robot.interfaces.feedback import Snapshot
from robocell_sim.robot.interfaces.position import Position

Acquisition = tuple[Snapshot, float]


class Recording(Protocol):
    """Encoder baseline advanced only from complete feedback."""

    def commit(self, moment: float) -> tuple[Position, tuple[Acquisition, ...]]:
        """Record a new sample without inventing travel across an epoch."""
        ...
