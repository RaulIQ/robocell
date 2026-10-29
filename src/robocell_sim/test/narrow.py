"""A serial endpoint that accepts only a short prefix on each write."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port


@final
@dataclass(frozen=True)
class Narrow(Port):
    """Port that collects accepted bytes and exposes partial-write behavior."""

    _accepted: bytearray
    _capacity: int

    @override
    def receive(self) -> bytes:
        return bytes(self._accepted)

    @override
    def send(self, payload: bytes) -> int:
        self._accepted.extend(payload[: self._capacity])
        return min(len(payload), self._capacity)
