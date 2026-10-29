"""A standard A1 request with a validated finite reply."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.robot.implementations.uart.exact import Exact
from robocell_sim.robot.implementations.uart.written import Written
from robocell_sim.robot.interfaces.transaction import Transaction


@final
@dataclass(frozen=True)
class Asked(Transaction):
    """Transaction that rejects an unexpected descriptor or reply length."""

    _port: Port
    _timeout: float

    @override
    def answer(self, command: bytes, descriptor: bytes, size: int) -> bytes:
        Written(self._port, command, self._timeout).publish()
        payload = Exact(
            self._port, len(descriptor) + size, self._timeout
        ).receive()
        if payload[: len(descriptor)] != descriptor:
            message = "RPLIDAR query returned an unexpected response descriptor"
            raise ValueError(message)
        return payload[len(descriptor) :]
