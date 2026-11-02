"""One independently supervised wheel endpoint."""

from typing import Protocol

import can

from robocell_sim.robot.interfaces.feedback import Snapshot
from robocell_sim.robot.interfaces.negotiation import Negotiation

Reading = tuple[Snapshot, float, float]


class Axis(Protocol):
    """Communication and coherent encoder feedback of one drive."""

    def advance(
        self, bus: can.BusABC, moment: float, *, synchronized: bool
    ) -> "Axis":
        """Advance the zero-motion initialization."""
        ...

    def read(self, message: can.Message, part: int, moment: float) -> "Axis":
        """Process one validated frame addressed to this wheel."""
        ...

    def ready(self, moment: float) -> tuple[bool, bool, bool]:
        """Report enable permission, heartbeat health and barrier readiness."""
        ...

    def restart(self, negotiation: Negotiation) -> "Axis":
        """Discard partial feedback and invalidate the encoder baseline."""
        ...

    def readings(self) -> tuple[Reading, ...]:
        """Compute the most recent snapshot, angle and reception time."""
        ...
