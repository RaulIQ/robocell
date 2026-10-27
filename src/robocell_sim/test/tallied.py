"""Journal that keeps its losses in memory."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.journal import Journal


@final
@dataclass(frozen=True)
class Tallied(Journal):
    """Journal that appends every count of lost measurements to a list.

    The list receives the counts in the order the pairings noted them, so
    a test reads what the driver would have reported.
    """

    _counts: list[int]

    @override
    def lose(self, count: int) -> None:
        self._counts.append(count)
