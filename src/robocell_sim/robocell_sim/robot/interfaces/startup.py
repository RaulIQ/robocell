"""A new stopped host epoch."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Session


class Startup(Protocol):
    """Validated identities and fresh initialization of both drives."""

    def session(self, moment: float, *, recovery: bool) -> Session:
        """Create a new five-second attempt without cached motion."""
        ...
