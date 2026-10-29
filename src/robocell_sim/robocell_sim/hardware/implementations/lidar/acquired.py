"""One timed return of the standard rotating scanner."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.implementations.lidar.encoded import Encoded
from robocell_sim.hardware.interfaces.beam import Beam
from robocell_sim.hardware.interfaces.packet import Packet


@final
@dataclass(frozen=True)
class Acquired(Packet):
    """Packet that ranges the live world at a scheduled motor angle.

    Timing holds the period in microseconds and frequency in turns per
    second. Sample holds the index and session start in simulated seconds.
    """

    _beam: Beam
    _timing: tuple[int, float]
    _sample: tuple[int, float]

    @override
    def encode(self) -> bytes:
        turn = (
            self._sample[1] + (self._sample[0] + 1) * self._timing[0] / 1e6
        ) * self._timing[1]
        previous = (
            self._sample[1] + self._sample[0] * self._timing[0] / 1e6
        ) * self._timing[1]
        angle = (turn % 1) * 360
        return Encoded(
            angle,
            self._beam.distance(angle),
            _start=self._sample[0] == 0
            or math.floor(turn) > math.floor(previous),
            _quality=15,
        ).encode()
