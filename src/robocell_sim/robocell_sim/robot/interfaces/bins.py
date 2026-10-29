"""Uniform angular bins for a scan."""

from typing import Protocol


class Bins(Protocol):
    """Bins that associate a range and quality with each angular slot."""

    def returns(self) -> dict[int, tuple[float, int]]:
        """Project received points into angular slots."""
        ...
