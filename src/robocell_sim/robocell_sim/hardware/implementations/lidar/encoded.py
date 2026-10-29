"""Standard RPLIDAR measurement in five bytes."""

import math
import struct
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.packet import Packet


@final
@dataclass(frozen=True)
class Encoded(Packet):
    """Packet that preserves the A1 angle, return and rotation boundary.

    Angle is clockwise in degrees and distance is in meters. Start marks
    the first measurement of a rotation and quality describes the echo.
    Unrepresentable returns carry zero distance and quality on the wire.
    """

    _angle: float
    _distance: float
    _start: bool
    _quality: int

    @override
    def encode(self) -> bytes:
        distance = (
            round(self._distance * 4000)
            if math.isfinite(self._distance)
            and 0 < self._distance <= 65535 / 4000
            else 0
        )
        return struct.pack(
            "<BHH",
            ((self._quality if distance else 0) << 2)
            | (int(not self._start) << 1)
            | int(self._start),
            ((round(self._angle * 64) % 23040) << 1) | 1,
            distance,
        )
