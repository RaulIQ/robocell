"""RPLIDAR host commands retained across serial reads."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.implementations.lidar.framed import Framed
from robocell_sim.hardware.interfaces.request import Request

_COMMANDS = (0x20, 0x21, 0x25, 0x40, 0x50, 0x52, 0x59)


@final
@dataclass(frozen=True)
class Requested(Request):
    """Request stream that skips garbage and whole unsupported commands."""

    _payload: bytes

    @override
    def split(self) -> tuple[tuple[int, ...], bytes]:
        payload = self._payload
        commands: list[int] = []
        while payload:
            payload = (
                payload[position:]
                if (position := payload.find(b"\xa5")) >= 0
                else b""
            )
            size = Framed(payload).size()
            if not size:
                break
            commands.extend((payload[1],) if payload[1] in _COMMANDS else ())
            payload = payload[size:]
        return (tuple(commands), payload)
