"""The standard-SCAN session of an emulated A1."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.implementations.lidar.answered import Answered
from robocell_sim.hardware.implementations.lidar.requested import Requested
from robocell_sim.hardware.implementations.lidar.rotated import Rotated
from robocell_sim.hardware.interfaces.beam import Beam
from robocell_sim.hardware.interfaces.device import Device

_TIMEOUT = 5.0


@final
@dataclass(frozen=True)
class Rplidar(Device):
    """Device session with a live beam, acquisition state and pending request.

    Scan holds activity, simulated start time and emitted sample count.
    Request holds incomplete bytes and their first arrival time. The
    scanner has an already powered motor rotating at 5.5 turns per second.
    """

    _beam: Beam
    _scan: tuple[bool, float, int]
    _request: tuple[bytes, float]

    @override
    def exchange(self, payload: bytes, moment: float) -> tuple[Device, bytes]:
        prefix = (
            self._request[0]
            if 0 <= moment - self._request[1] <= _TIMEOUT
            else b""
        )
        commands, pending = Requested(prefix + payload).split()
        scan = self._scan
        replies: list[bytes] = []
        for command in commands:
            scan = (command in {0x20, 0x21}, moment, 0)
            replies.append(Answered(command).encode())
        count, samples = (
            Rotated(self._beam, 476, 5.5).advance(moment, scan[1], scan[2])
            if scan[0]
            else (scan[2], b"")
        )
        return (
            Rplidar(
                self._beam,
                (scan[0], scan[1], count),
                (pending, self._request[1] if prefix and pending else moment),
            ),
            b"".join(replies) + samples,
        )
