"""A finite serial reply read incrementally."""

import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.robot.interfaces.response import Response


@final
@dataclass(frozen=True)
class Exact(Response):
    """Response that accepts partial reads without an unbounded wait."""

    _port: Port
    _size: int
    _timeout: float

    @override
    def receive(self) -> bytes:
        deadline = time.monotonic() + self._timeout
        payload = b""
        while len(payload) < self._size and time.monotonic() < deadline:
            payload += self._port.receive()
            if len(payload) < self._size:
                time.sleep(0.001)
        if len(payload) != self._size:
            message = (
                "RPLIDAR response did not match its expected size "
                "before the deadline"
            )
            raise TimeoutError(message)
        return payload
