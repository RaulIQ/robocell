"""The exact configuration checks required by profile revision one."""

import struct
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.motor.negotiated import Negotiated
from robocell_sim.robot.interfaces.negotiation import Exchange, Negotiation
from robocell_sim.robot.interfaces.plan import Plan


@final
@dataclass(frozen=True)
class Planned(Plan):
    """Plan that verifies wire units, fixed PDO maps and all supervision."""

    _number: int

    @override
    def negotiation(self, *, recovery: bool) -> Negotiation:
        checks = (
            (0x1000, 0, 4, 0x192),
            (0x2003, 0, 4, 1),
            (0x6060, 0, 1, 3),
            (0x6061, 0, 1, 3),
            (0x6502, 0, 4, 4),
            (0x608F, 0, 1, 2),
            (0x608F, 1, 4, 4096),
            (0x608F, 2, 4, 1),
            (0x1016, 0, 1, 1),
            (0x1016, 1, 4, 0x7F012C),
            (0x1017, 0, 2, 100),
            (0x2002, 0, 2, 200),
            (0x1400, 0, 1, 2),
            (0x1400, 1, 4, 0x200 + self._number),
            (0x1400, 2, 1, 255),
            *(
                (0x1800 + offset, subindex, size, value)
                for offset, base in enumerate((0x180, 0x280, 0x380))
                for subindex, size, value in (
                    (0, 1, 5),
                    (1, 4, base + self._number),
                    (2, 1, 255),
                    (3, 2, 0),
                    (5, 2, 20),
                )
            ),
            *(
                (index, subindex, 1 if subindex == 0 else 4, value)
                for index, values in (
                    (0x1600, (2, 0x60400010, 0x60FF0020)),
                    (0x1A00, (3, 0x60410010, 0x60640020, 0x20000010)),
                    (0x1A01, (2, 0x606C0020, 0x20000010)),
                    (0x1A02, (2, 0x20010120, 0x20010220)),
                )
                for subindex, value in enumerate(values)
            ),
        )
        steps: tuple[Exchange, ...] = (
            (0, bytes((130, self._number)), b""),
            (0x700 + self._number, b"", b"\x7f"),
            *(
                (
                    (0, bytes((1, self._number)), b""),
                    (0x700 + self._number, b"", b"\x05"),
                    (0x200 + self._number, struct.pack("<Hi", 0, 0), b""),
                    (0x200 + self._number, struct.pack("<Hi", 128, 0), b""),
                    (
                        0x600 + self._number,
                        bytes.fromhex("40 41 60 00 00 00 00 00"),
                        struct.pack("<BHBH2x", 0x4B, 0x6041, 0, 64),
                    ),
                    (0x200 + self._number, struct.pack("<Hi", 0, 0), b""),
                )
                if recovery
                else ()
            ),
            (0, bytes((128, self._number)), b""),
            (
                0x600 + self._number,
                bytes.fromhex("2F 60 60 00 03 00 00 00"),
                bytes.fromhex("60 60 60 00 00 00 00 00"),
            ),
            *(
                (
                    0x600 + self._number,
                    struct.pack("<BHB4x", 0x40, index, subindex),
                    struct.pack(
                        "<BHB4s",
                        {1: 0x4F, 2: 0x4B, 4: 0x43}[size],
                        index,
                        subindex,
                        value.to_bytes(size, "little").ljust(4, b"\0"),
                    ),
                )
                for index, subindex, size, value in checks
            ),
            (0, bytes((1, self._number)), b""),
            (0x700 + self._number, b"", b"\x05"),
            *(
                step
                for word, status in (
                    (6, 33),
                    (7, 35),
                    (15, 39),
                )
                for step in (
                    (0x200 + self._number, struct.pack("<Hi", word, 0), b""),
                    (
                        0x600 + self._number,
                        bytes.fromhex("40 41 60 00 00 00 00 00"),
                        struct.pack("<BHBH2x", 0x4B, 0x6041, 0, status),
                    ),
                )
            ),
        )
        return Negotiated(self._number, steps, (-1.0, -1.0, 0), (0, -1.0))
