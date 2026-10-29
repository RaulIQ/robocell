"""A decoded standard A1 measurement."""

from typing import Protocol


class Echo(Protocol):
    """Echo whose structural validity is separate from a missing return."""

    def valid(self) -> bool:
        """Check framing bits and the representable angular interval."""
        ...

    def values(self) -> tuple[bool, float, float, int]:
        """Return rotation start, clockwise degrees, meters and quality."""
        ...
