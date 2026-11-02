"""Matched wheel snapshots and protective feedback checks."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Joints


class Observation(Protocol):
    """Coherent paired feedback and independent drive supervision."""

    def joints(self, moment: float) -> tuple[Joints, ...]:
        """Pair acquisitions within one physics step and expire old samples."""
        ...

    def verify(self, moment: float, clock: int) -> None:
        """Reject lost heartbeat, stale advancing feedback and fault status."""
        ...

    def disabled(self) -> bool:
        """Whether a fresh snapshot exposes inhibited operation."""
        ...
