"""Bounded expedited SDO access for the wheel profile."""

import struct
from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.implementations.motor.permitted import Permitted
from robocell_sim.hardware.interfaces.access import Access

_UPLOAD = 0x40
_RECORDS = {
    0x1016,
    0x1400,
    0x1600,
    0x1800,
    0x1801,
    0x1802,
    0x1A00,
    0x1A01,
    0x1A02,
    0x2001,
    0x608F,
}
_WRITABLE = {(0x1016, 1), (0x1017, 0), (0x2002, 0), (0x6060, 0)}


@final
@dataclass(frozen=True)
class Accessible(Access):
    """Access that rejects unsupported transfers before library dispatch."""

    _node: canopen.LocalNode
    _payload: bytes
    _status: int

    @override
    def code(self) -> int:
        command, index, subindex = struct.unpack_from("<BHB", self._payload)
        code = 0
        if command not in {_UPLOAD, 0x2F, 0x2B, 0x23}:
            code = 0x05040001
        elif (
            index in self._node.object_dictionary
            and index not in _RECORDS
            and subindex != 0
        ):
            code = 0x06090011
        elif command != _UPLOAD and (index, subindex) in _WRITABLE:
            code = Permitted(self._node, self._payload, self._status).code()
        return code
