"""Bus that transmits frames through python-can."""

from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.hardware.interfaces.bus import Bus


@final
@dataclass(frozen=True)
class Transmitted(Bus):
    """Bus that hands every frame to a python-can bus.

    The origin is an open python-can bus, SocketCAN on vcan0 in the
    simulator or a virtual one in tests, and each frame goes out on it as
    a classic CAN frame with a standard identifier.
    """

    _origin: can.BusABC

    @override
    def send(self, identifier: int, payload: bytes) -> None:
        self._origin.send(
            can.Message(
                arbitration_id=identifier,
                data=payload,
                is_extended_id=False,
            )
        )
