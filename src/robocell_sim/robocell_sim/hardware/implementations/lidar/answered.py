"""Standard replies of the synthetic A1M8 board."""

import struct
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.packet import Packet


@final
@dataclass(frozen=True)
class Answered(Packet):
    """Reply for the standard-SCAN profile with a synthetic board identity.

    Standard samples take 476 microseconds, matching the SDK legacy
    profile. The 125-microsecond express period describes the hardware;
    express acquisition itself is outside this emulator's supported modes.
    """

    _command: int

    @override
    def encode(self) -> bytes:
        return {
            0x20: bytes.fromhex("A5 5A 05 00 00 40 81"),
            0x21: bytes.fromhex("A5 5A 05 00 00 40 81"),
            0x25: b"",
            0x40: b"",
            0x50: bytes.fromhex("A5 5A 14 00 00 00 04 18 11 01 05")
            + b"ROBOCELL-A1-0001",
            0x52: bytes.fromhex("A5 5A 03 00 00 00 06 00 00 00"),
            0x59: bytes.fromhex("A5 5A 04 00 00 00 15")
            + struct.pack("<HH", 476, 125),
        }[self._command]
