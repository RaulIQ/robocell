"""Continuous encoder position across signed wire rollover."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.position import Position

_GAP = 300000000


@final
@dataclass(frozen=True)
class Unwrapped(Position):
    """Position retaining travel while invalidating discontinuous baselines.

    The reading holds the previous wire count, accumulated count and time.
    An initial time of minus one adopts the first absolute wire position.
    """

    _reading: tuple[int, int, int]
    _valid: bool

    @override
    def read(self, position: int, stamp: int) -> Position:
        previous, total, instant = self._reading
        distance = (position - previous + 2**31) % 2**32 - 2**31
        if instant == -1:
            total = position
        elif (
            self._valid and 0 < stamp - instant <= _GAP and distance != -(2**31)
        ):
            total += distance
        return Unwrapped((position, total, stamp), _valid=True)

    @override
    def reset(self) -> Position:
        return Unwrapped(self._reading, _valid=False)

    @override
    def angle(self) -> float:
        return self._reading[1] * 2 * math.pi / 4096
