"""Coherent assembly of fixed CANopen PDO feedback."""

import struct
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.feedback import Feedback, Snapshot

_EXPIRY = 0.1
_LENGTHS = {1: 8, 2: 6, 3: 8}
_PARTS = 3


@final
@dataclass(frozen=True)
class Assembled(Feedback):
    """Snapshot that commits only a timely, ordered PDO triple."""

    _frames: tuple[bytes, ...]
    _moment: float

    @override
    def read(self, part: int, payload: bytes, moment: float) -> Feedback:
        frames = self._frames if moment - self._moment <= _EXPIRY else ()
        result = Assembled((), moment)
        if len(payload) == _LENGTHS.get(part) and part == 1:
            result = Assembled((payload,), moment)
        elif (
            len(payload) == _LENGTHS.get(part)
            and len(frames) == part - 1
            and (part == _PARTS or frames[0][6:8] == payload[4:6])
        ):
            result = Assembled((*frames, payload), self._moment)
        return result

    @override
    def complete(self) -> bool:
        return len(self._frames) == _PARTS

    @override
    def measurement(self) -> Snapshot:
        if not self.complete():
            error = "Drive feedback is incomplete without all three PDOs"
            raise ValueError(error)
        status, position, sample = struct.unpack("<HiH", self._frames[0])
        return (
            status,
            position,
            struct.unpack("<iH", self._frames[1])[0],
            sample,
            int.from_bytes(self._frames[2], "little"),
        )
