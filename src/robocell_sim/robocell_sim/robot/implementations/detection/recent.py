"""Age limits for observations on a robot's clock."""

from dataclasses import dataclass
from typing import final

from robocell_sim.robot.interfaces.clock import Clock


@final
@dataclass(frozen=True)
class Recent:
    """Accept acquisition timestamps within a bounded age in nanoseconds."""

    _clock: Clock
    _limit: int = 500_000_000

    def __call__(self, stamp: int) -> bool:
        """Check that the observation has arrived and is still usable."""
        return 0 <= self._clock.now() - stamp <= self._limit
