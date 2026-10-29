"""Sequential samples from a rotating A1 standard scanner."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.implementations.lidar.acquired import Acquired
from robocell_sim.hardware.interfaces.beam import Beam
from robocell_sim.hardware.interfaces.rotation import Rotation

_LIMIT = 4096


@final
@dataclass(frozen=True)
class Rotated(Rotation):
    """Rotation that samples a live beam at a fixed period and spin rate.

    Period is in microseconds and frequency is in turns per second. Each
    due sample sees the current physics state, with at most one physics
    step of acquisition quantization when several rays fall in that step.
    """

    _beam: Beam
    _period: int
    _frequency: float

    @override
    def advance(
        self, moment: float, start: float, count: int
    ) -> tuple[int, bytes]:
        due = max(
            count, math.floor((moment - start) * 1e6 / self._period + 1e-7)
        )
        if due - count > _LIMIT:
            message = "RPLIDAR acquisition missed too many physics steps"
            raise BufferError(message)
        return (
            due,
            b"".join(
                Acquired(
                    self._beam, (self._period, self._frequency), (index, start)
                ).encode()
                for index in range(count, due)
            ),
        )
