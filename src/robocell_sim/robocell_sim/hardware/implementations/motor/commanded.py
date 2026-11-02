"""Atomic command PDO reception."""

import struct
from dataclasses import dataclass
from typing import final, override

import can
import canopen

from robocell_sim.hardware.interfaces.exchange import Exchange
from robocell_sim.hardware.interfaces.session import Session
from robocell_sim.hardware.interfaces.shaft import Shaft

_WORDS = {0, 2, 6, 7, 15, 128}
_LIMIT = 7880


@final
@dataclass(frozen=True)
class Commanded(Exchange):
    """PDO command that requires live communication and physical permission."""

    _node: canopen.LocalNode
    _shaft: Shaft
    _session: Session

    @override
    def exchange(self, message: can.Message, moment: float) -> Session:
        motor = self._session[0]
        word, target = struct.unpack("<Hi", message.data)
        timeout = (
            int.from_bytes(self._node.get_data(0x1016, 1), "little") & 0xFFFF
        )
        if (
            self._node.nmt.state == "OPERATIONAL"
            and word in _WORDS
            and -_LIMIT <= target <= _LIMIT
        ):
            motor = motor.command(
                word,
                target,
                moment,
                ready=not self._shaft.paused()
                and moment - self._session[1][0] < timeout / 1000,
            )
            self._node.set_data(0x6040, 0, word.to_bytes(2, "little"))
        return (motor, self._session[1], self._session[2])
