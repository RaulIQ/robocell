"""Length of a standard or payload-bearing RPLIDAR request."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.frame import Frame

_HEADER = 2
_PAYLOAD = 3


@final
@dataclass(frozen=True)
class Framed(Frame):
    """Frame that consumes unsupported payload commands without their contents.

    The payload starts at A5. Its command's high bit selects the extended
    request layout, preventing embedded bytes from becoming STOP or SCAN.
    """

    _payload: bytes

    @override
    def size(self) -> int:
        if len(self._payload) < _HEADER:
            return 0
        size = (
            (self._payload[2] + 4 if len(self._payload) >= _PAYLOAD else 0)
            if self._payload[1] & 0x80
            else 2
        )
        return size if len(self._payload) >= size else 0
