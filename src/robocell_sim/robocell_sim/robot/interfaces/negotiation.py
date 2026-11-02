"""Bounded initialization of one CANopen velocity drive."""

from typing import Protocol

import can

Exchange = tuple[int, bytes, bytes]


class Negotiation(Protocol):
    """Verified configuration and zero-target CiA 402 enable sequence."""

    def advance(
        self, bus: can.BusABC, moment: float, *, synchronized: bool
    ) -> "Negotiation":
        """Send or retry the pending protocol exchange without blocking."""
        ...

    def read(self, message: can.Message, moment: float) -> "Negotiation":
        """Accept a matching response and independently track heartbeat."""
        ...

    def enabled(self) -> bool:
        """Whether all configuration and transition checks succeeded."""
        ...

    def healthy(self, moment: float) -> bool:
        """Whether the drive has a fresh operational heartbeat."""
        ...

    def waiting(self) -> bool:
        """Whether this drive awaits the shared NMT-start barrier."""
        ...
