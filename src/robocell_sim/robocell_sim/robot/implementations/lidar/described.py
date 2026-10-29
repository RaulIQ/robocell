"""A bounded standard-SCAN descriptor and its trailing payload."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.descriptor import Descriptor

_LIMIT = 65536
_SIZE = 7


@final
@dataclass(frozen=True)
class Described(Descriptor):
    """Descriptor that accepts only the standard five-byte stream format."""

    _payload: bytes
    _waiting: bool

    @override
    def payload(self) -> tuple[bytes, bool]:
        if len(self._payload) > _LIMIT:
            message = "RPLIDAR receive buffer exceeded its byte limit"
            raise BufferError(message)
        if not self._waiting or len(self._payload) < _SIZE:
            return (self._payload, self._waiting)
        if self._payload[:_SIZE] != bytes.fromhex("A5 5A 05 00 00 40 81"):
            message = "RPLIDAR SCAN response has an invalid descriptor"
            raise ValueError(message)
        return (self._payload[_SIZE:], False)
