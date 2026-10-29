"""Host commands extracted from a serial chunk."""

from typing import Protocol


class Request(Protocol):
    """Request stream that retains an incomplete trailing command."""

    def split(self) -> tuple[tuple[int, ...], bytes]:
        """Return supported commands and the incomplete suffix."""
        ...
