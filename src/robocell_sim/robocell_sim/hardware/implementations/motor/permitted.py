"""Configuration write permission and range validation."""

import struct
from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.interfaces.access import Access

_HOST = 127
_CONSUMER = (250, 1000)
_PRODUCER = (50, 200)
_WATCHDOG = (100, 500)
_MODE = 3
_DISABLED = 0x40
_SIZES = {0x1016: 4, 0x1017: 2, 0x2002: 2, 0x6060: 1}


@final
@dataclass(frozen=True)
class Permitted(Access):
    """Permission that validates configuration writes before storage."""

    _node: canopen.LocalNode
    _payload: bytes
    _status: int

    @override
    def code(self) -> int:
        command, index, _ = struct.unpack_from("<BHB", self._payload)
        size = {0x2F: 1, 0x2B: 2, 0x23: 4}[command]
        value = int.from_bytes(self._payload[4 : 4 + size], "little")
        valid = {
            0x1016: value >> 16 == _HOST
            and _CONSUMER[0] <= (value & 0xFFFF) <= _CONSUMER[1],
            0x1017: _PRODUCER[0] <= value <= _PRODUCER[1],
            0x2002: _WATCHDOG[0] <= value <= _WATCHDOG[1],
            0x6060: value == _MODE,
        }[index]
        code = 0 if valid else 0x06090030
        if (
            self._node.nmt.state != "PRE-OPERATIONAL"
            or self._status != _DISABLED
        ):
            code = 0x08000022
        if size != _SIZES[index]:
            code = 0x06070010
        return code
