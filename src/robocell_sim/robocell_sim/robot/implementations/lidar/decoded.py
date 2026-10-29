"""A standard five-byte A1 return."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.echo import Echo

_TURN = 23040


@final
@dataclass(frozen=True)
class Decoded(Echo):
    """Echo decoded independently from the device emulator's encoding."""

    _payload: bytes

    @override
    def valid(self) -> bool:
        return (
            bool((self._payload[0] ^ (self._payload[0] >> 1)) & 1)
            and bool(self._payload[1] & 1)
            and (int.from_bytes(self._payload[1:3], "little") >> 1) < _TURN
        )

    @override
    def values(self) -> tuple[bool, float, float, int]:
        return (
            bool(self._payload[0] & 1),
            (int.from_bytes(self._payload[1:3], "little") >> 1) / 64,
            int.from_bytes(self._payload[3:5], "little") / 4000,
            self._payload[0] >> 2,
        )
