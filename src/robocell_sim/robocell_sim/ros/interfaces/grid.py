"""Angular bins containing range and return strength."""

from typing import Protocol


class Grid(Protocol):
    """Nearest valid obstacle at each occupied angular bin."""

    def returns(self) -> dict[int, tuple[float, float]]:
        """Return occupied bins with their distances and strengths."""
        ...
