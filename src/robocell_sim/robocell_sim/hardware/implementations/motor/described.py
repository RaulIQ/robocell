"""Object dictionary for the bounded wheel drive profile."""

from dataclasses import dataclass
from typing import final, override

import canopen
from canopen.objectdictionary import (
    INTEGER8,
    INTEGER32,
    UNSIGNED8,
    UNSIGNED16,
    UNSIGNED32,
    ObjectDictionary,
)

from robocell_sim.hardware.implementations.motor.defined import Defined
from robocell_sim.hardware.implementations.motor.grouped import Grouped
from robocell_sim.hardware.interfaces.dictionary import Dictionary

_HOST = 127


@final
@dataclass(frozen=True)
class Described(Dictionary):
    """Description that gives a node its fixed PDO maps and typed objects."""

    _number: int

    @override
    def node(self) -> canopen.LocalNode:
        if not 1 <= self._number < _HOST:
            error = "Wheel node must be between 1 and 126"
            raise ValueError(error)
        dictionary = ObjectDictionary()
        for index, datatype, value, access in (
            (0x1000, UNSIGNED32, 0x192, "ro"),
            (0x1001, UNSIGNED8, 0, "ro"),
            (0x1017, UNSIGNED16, 100, "rw"),
            (0x2000, UNSIGNED16, 0, "ro"),
            (0x2002, UNSIGNED16, 200, "rw"),
            (0x2003, UNSIGNED32, 1, "ro"),
            (0x603F, UNSIGNED16, 0, "ro"),
            (0x6040, UNSIGNED16, 0, "ro"),
            (0x6041, UNSIGNED16, 0x40, "ro"),
            (0x6060, INTEGER8, 3, "rw"),
            (0x6061, INTEGER8, 3, "ro"),
            (0x6064, INTEGER32, 0, "ro"),
            (0x606C, INTEGER32, 0, "ro"),
            (0x60FF, INTEGER32, 0, "ro"),
            (0x6502, UNSIGNED32, 4, "ro"),
        ):
            dictionary.add_object(
                Defined((index, 0), (datatype, value, access)).object()
            )
        for index, values in (
            (0x1016, ((1, UNSIGNED32, 0x7F012C, "rw"),)),
            (
                0x1400,
                (
                    (1, UNSIGNED32, 0x200 + self._number, "ro"),
                    (2, UNSIGNED8, 255, "ro"),
                ),
            ),
            (
                0x1600,
                (
                    (1, UNSIGNED32, 0x60400010, "ro"),
                    (2, UNSIGNED32, 0x60FF0020, "ro"),
                ),
            ),
            (
                0x1A00,
                (
                    (1, UNSIGNED32, 0x60410010, "ro"),
                    (2, UNSIGNED32, 0x60640020, "ro"),
                    (3, UNSIGNED32, 0x20000010, "ro"),
                ),
            ),
            (
                0x1A01,
                (
                    (1, UNSIGNED32, 0x606C0020, "ro"),
                    (2, UNSIGNED32, 0x20000010, "ro"),
                ),
            ),
            (
                0x1A02,
                (
                    (1, UNSIGNED32, 0x20010120, "ro"),
                    (2, UNSIGNED32, 0x20010220, "ro"),
                ),
            ),
            (
                0x2001,
                (
                    (1, UNSIGNED32, 0, "ro"),
                    (2, UNSIGNED32, 0, "ro"),
                ),
            ),
            (
                0x608F,
                (
                    (1, UNSIGNED32, 4096, "ro"),
                    (2, UNSIGNED32, 1, "ro"),
                ),
            ),
            *(
                (
                    0x1800 + offset,
                    (
                        (1, UNSIGNED32, base + self._number, "ro"),
                        (2, UNSIGNED8, 255, "ro"),
                        (3, UNSIGNED16, 0, "ro"),
                        (5, UNSIGNED16, 20, "ro"),
                    ),
                )
                for offset, base in enumerate((0x180, 0x280, 0x380))
            ),
        ):
            dictionary.add_object(Grouped(index, values).object())
        return canopen.LocalNode(self._number, dictionary)
