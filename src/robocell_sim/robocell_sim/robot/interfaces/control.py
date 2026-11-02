"""One aged body-command path for every operator source."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Command


class Control(Protocol):
    """Application input whose age survives collision-guard reevaluation."""

    def steer(self, velocity: tuple[float, float], moment: float) -> "Control":
        """Accept a newer source command with its original monotonic time."""
        ...

    def scan(self, ranges: tuple[float, ...], stamp: int) -> "Control":
        """Replace guard measurements without renewing application input."""
        ...

    def reading(self, clock: int, *, protected: bool) -> Command:
        """Return guarded body velocity with the unchanged source time."""
        ...
