"""A serial endpoint whose peer must remain connected."""

import select
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port


@final
@dataclass(frozen=True)
class Connected(Port):
    """Port that distinguishes a quiet device from a hung-up terminal."""

    _origin: Port
    _descriptor: int

    @override
    def receive(self) -> bytes:
        poll = select.poll()
        poll.register(
            self._descriptor, select.POLLIN | select.POLLHUP | select.POLLERR
        )
        if any(
            flags & (select.POLLHUP | select.POLLERR)
            for _, flags in poll.poll(0)
        ):
            message = "RPLIDAR serial endpoint disconnected"
            raise ConnectionError(message)
        return self._origin.receive()

    @override
    def send(self, payload: bytes) -> int:
        return self._origin.send(payload)
