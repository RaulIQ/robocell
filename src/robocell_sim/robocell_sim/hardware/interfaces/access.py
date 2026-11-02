"""Permission for one SDO transaction."""

from typing import Protocol


class Access(Protocol):
    """Access decision expressed as a CANopen abort code, or zero."""

    def code(self) -> int:
        """Return zero for allowed access or its precise abort code."""
        ...
