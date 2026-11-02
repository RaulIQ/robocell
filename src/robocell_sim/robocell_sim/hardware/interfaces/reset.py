"""Reset of a drive's volatile communication session."""

from typing import Protocol

from robocell_sim.hardware.interfaces.session import Session


class Reset(Protocol):
    """Reset that inhibits motion before announcing a fresh boot."""

    def reset(self, command: int, moment: float) -> Session:
        """Apply NMT node or communication reset."""
        ...
