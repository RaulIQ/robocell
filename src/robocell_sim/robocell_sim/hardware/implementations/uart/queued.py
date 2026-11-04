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
    Idle credit is limited to 256 bytes; previously queued bytes retain
    the transmission time that elapsed while physics was busy.
    """

    _port: Port
    _payload: bytes
    _stamp: float
    _limit: int

    @override
    def advance(self, moment: float, backlog: int) -> tuple[bytes, float]:
        if len(self._payload) > self._limit:
            message = "RPLIDAR UART backlog exceeded its byte limit"
            raise BufferError(message)
        stamp = max(self._stamp, moment - max(256, backlog) / 11520)
        budget = max(
            0, min(max(256, backlog), int((moment - self._stamp) * 11520))
        )
        count = self._port.send(self._payload[:budget]) if budget else 0
        return (
            self._payload[count:],
            stamp + count / 11520 if count < len(self._payload) else moment,
        )
