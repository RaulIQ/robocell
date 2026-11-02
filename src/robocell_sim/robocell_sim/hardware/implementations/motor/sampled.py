"""Coherent shaft acquisition and fixed feedback PDOs."""

import math
import struct
from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.hardware.interfaces.shaft import Shaft


@final
@dataclass(frozen=True)
class Sampled(Report):
    """Snapshot whose position and velocity come from the physical shaft."""

    _node: canopen.LocalNode
    _shaft: Shaft
    _number: int

    @override
    def publish(self) -> None:
        angle, velocity, stamp = self._shaft.measurement()
        position = (
            math.floor(angle * 4096 / (2 * math.pi)) + 2**31
        ) % 2**32 - 2**31
        speed = math.floor(abs(velocity) * 1000 + 0.5) * (
            -1 if velocity < 0 else 1
        )
        instant = round(stamp * 1e9)
        for index, subindex, payload in (
            (0x6064, 0, struct.pack("<i", position)),
            (0x606C, 0, struct.pack("<i", speed)),
            (0x2000, 0, struct.pack("<H", self._number)),
            (0x2001, 1, struct.pack("<I", instant & 0xFFFFFFFF)),
            (0x2001, 2, struct.pack("<I", instant >> 32)),
        ):
            self._node.set_data(index, subindex, payload)
        for identifier, payload in (
            (
                0x180,
                self._node.get_data(0x6041, 0)
                + struct.pack("<iH", position, self._number),
            ),
            (0x280, struct.pack("<iH", speed, self._number)),
            (0x380, struct.pack("<Q", instant)),
        ):
            self._node.network.send_message(identifier + self._node.id, payload)
