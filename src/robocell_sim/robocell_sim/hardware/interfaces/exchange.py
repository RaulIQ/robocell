"""A CAN frame applied to a drive session."""

from typing import Protocol

import can

from robocell_sim.hardware.interfaces.session import Session


class Exchange(Protocol):
    """Exchange that validates one received frame before applying it."""

    def exchange(self, message: can.Message, moment: float) -> Session:
        """Apply a frame at monotonic reception time."""
        ...
