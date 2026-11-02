"""Best-effort protective commands for both drives."""

import contextlib
import struct
from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.interfaces.stop import Stop


@final
@dataclass(frozen=True)
class Stopped(Stop):
    """Stop that never lets an unavailable wheel prevent its peer's stop."""

    _bus: can.BusABC
    _numbers: tuple[int, int]

    @override
    def apply(self, word: int) -> None:
        for number in self._numbers:
            with contextlib.suppress(can.CanError, OSError):
                self._bus.send(
                    can.Message(
                        arbitration_id=0x200 + number,
                        data=struct.pack("<Hi", word, 0),
                        is_extended_id=False,
                    ),
                    timeout=0.01,
                )
