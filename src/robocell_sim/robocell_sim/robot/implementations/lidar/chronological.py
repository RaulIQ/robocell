"""Continuous acquisition timing across fragmented serial reads."""

import logging
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.chronology import Chronology


@final
@dataclass(frozen=True)
class Chronological(Chronology):
    """Chronology anchored once and advanced by acquired sample durations.

    Boundary is the next turn's start in nanoseconds, negative when an
    arrival estimate is needed. Floor protects already published intervals
    when damaged bytes or contradictory timing require a new anchor.
    """

    _boundary: int
    _floor: int

    @override
    def read(
        self, first: int, duration: int, ceiling: int
    ) -> tuple[Chronology, tuple[int, ...]]:
        stamp = (
            self._boundary if self._boundary >= 0 else max(first, self._floor)
        )
        if stamp + duration > ceiling:
            logging.getLogger(__name__).warning(
                "RPLIDAR discarded a turn ending at %s ns "
                "beyond ROS time %s ns",
                stamp + duration,
                ceiling,
            )
            return (self.reset(), ())
        return (
            Chronological(stamp + duration, stamp + duration),
            (stamp,),
        )

    @override
    def reset(self) -> Chronology:
        return Chronological(-1, self._floor)
