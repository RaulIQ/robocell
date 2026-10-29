"""The state of a serial scan receiver."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.sweep import Sweep


class Reception(Protocol):
    """Reception that assembles immutable scans from a fragmented stream."""

    def read(
        self, payload: bytes, stamp: int, moment: float
    ) -> tuple[Reception, tuple[Sweep, ...]]:
        """Consume received bytes with ROS time and monotonic arrival time."""
        ...
