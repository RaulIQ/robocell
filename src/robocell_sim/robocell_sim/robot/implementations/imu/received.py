"""Frame of the IMU board as it came off the CAN bus."""

import struct
from dataclasses import dataclass
from typing import TYPE_CHECKING, final, override

from robocell_sim.robot.interfaces.frame import Frame

if TYPE_CHECKING:
    from can.typechecking import CanFilters

GYRO = 0x101
ACCELEROMETER = 0x102
LAYOUT = struct.Struct("<hhhBx")
NUMBERS = 256
FILTERS: "CanFilters" = (
    {"can_id": GYRO, "can_mask": 0x7FF, "extended": False},
    {"can_id": ACCELEROMETER, "can_mask": 0x7FF, "extended": False},
)


@final
@dataclass(frozen=True)
class Received(Frame):
    """Frame that reads the bytes of docs/protocols/imu.md.

    The identifier tells the gyro frame from the accelerometer frame, and
    the payload holds x, y and z as little-endian int16, the measurement
    number in byte 6 and a reserved byte. The driver writes these numbers
    down from the protocol itself, not from the board emulator, as a driver
    for real hardware would from its datasheet.
    """

    _identifier: int
    _payload: bytes

    @override
    def gyro(self) -> bool:
        return self._identifier == GYRO

    @override
    def number(self) -> int:
        return int(LAYOUT.unpack(self._payload)[3])

    @override
    def values(self) -> tuple[int, int, int]:
        x, y, z, _ = LAYOUT.unpack(self._payload)
        return (int(x), int(y), int(z))

    @override
    def gap(self, last: int) -> int:
        return (self.number() - last - 1) % NUMBERS
