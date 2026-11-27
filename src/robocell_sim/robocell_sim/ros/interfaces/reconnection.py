"""Operations for starting and polling a drive connection."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Pilot


class Reconnection(Protocol):
    """Reconnect the active drive pilot within the operator's retry window."""

    def connect(self, moment: float, pilots: list[Pilot]) -> None:
        """Attempt a connection when the retry interval allows it."""
        ...

    def open(
        self, pilots: list[Pilot], deadline: float, *, recovery: bool
    ) -> None:
        """Open the verified drive connection and report failures."""
        ...

    def expired(self, moment: float) -> bool:
        """Check whether the active retry deadline has elapsed."""
        ...
