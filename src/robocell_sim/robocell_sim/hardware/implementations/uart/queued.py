"""Output paced to a 115200-baud 8N1 serial link."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.delivery import Delivery
from robocell_sim.hardware.interfaces.port import Port


@final
@dataclass(frozen=True)
class Queued(Delivery):
    """Delivery that preserves partial writes within a bounded backlog.

    The port accepts bytes, payload is the pending tail, stamp tracks wire
    credit in monotonic wall time, and limit bounds outstanding bytes.
    Credit never exceeds 256 bytes, including after long idle periods.
    """

    _port: Port
    _payload: bytes
    _stamp: float
    _limit: int

    @override
    def advance(self, moment: float) -> tuple[bytes, float]:
        if len(self._payload) > self._limit:
            message = "RPLIDAR UART backlog exceeded its byte limit"
            raise BufferError(message)
        stamp = max(self._stamp, moment - 256 / 11520)
        budget = max(0, int((moment - stamp) * 11520))
        count = self._port.send(self._payload[:budget]) if budget else 0
        return (
            self._payload[count:],
            stamp + count / 11520 if count < len(self._payload) else moment,
        )
