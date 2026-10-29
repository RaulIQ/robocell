"""A fixed clock for a serial publication scenario."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.clock import Clock


@final
@dataclass(frozen=True)
class Instant(Clock):
    """Clock held at an explicit instant during a deterministic exchange."""

    _seconds: float

    @override
    def now(self) -> int:
        return round(self._seconds * 1e9)
