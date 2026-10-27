"""Traffic on a CAN bus."""

from typing import Protocol


class Traffic(Protocol):
    """Traffic that a listener on a CAN bus hears."""

    def frames(self) -> tuple[tuple[int, bytes], ...]:
        """Return the identifier and payload of every frame heard, in order."""
        ...
