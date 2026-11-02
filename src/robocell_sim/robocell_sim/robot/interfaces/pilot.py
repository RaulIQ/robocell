"""Host side of a pair of synthetic CANopen velocity drives."""

from typing import Protocol

from robocell_sim.robot.interfaces.axis import Axis

Joints = tuple[int, tuple[float, float], tuple[float, float]]
Command = tuple[float, float, float]
Session = tuple[
    tuple[Axis, ...],
    tuple[str, float, bool],
    tuple[float, float, float, int],
    Command,
]


class Pilot(Protocol):
    """Drive supervision with independently aged application commands."""

    def advance(
        self, moment: float, clock: int
    ) -> tuple[str, tuple[Joints, ...]]:
        """Service the bus and return status with coherent wheel feedback."""
        ...

    def command(self, velocity: tuple[float, float], moment: float) -> None:
        """Accept a body command with its original monotonic arrival time."""
        ...

    def retry(self, moment: float, *, recovery: bool) -> None:
        """Authorize one bounded restart, optionally including fault reset."""
        ...

    def close(self) -> None:
        """Quick-stop both nodes and disable voltage within a bounded wait."""
        ...
