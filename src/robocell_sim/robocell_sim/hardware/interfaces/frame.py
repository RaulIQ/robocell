"""A command boundary in a binary stream."""

from typing import Protocol


class Frame(Protocol):
    """Frame that locates the end of a complete host request."""

    def size(self) -> int:
        """Return the full request length, or zero while incomplete."""
        ...
