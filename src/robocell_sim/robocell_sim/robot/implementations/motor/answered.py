"""Strict SDO response checking and bounded state confirmation."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.negotiation import Exchange
from robocell_sim.robot.interfaces.reply import Reply
from robocell_sim.robot.interfaces.transmission import Progress

_STATUS = bytes.fromhex("41 60 00")
_FAULTS = {8, 15}
_TRANSITION = 0.5


@final
@dataclass(frozen=True)
class Answered(Reply):
    """Answer separating unfinished transitions from invalid profiles."""

    _number: int
    _steps: tuple[Exchange, ...]
    _pending: tuple[float, float, int]

    @override
    def accept(self, payload: bytes, moment: float) -> Progress:
        expected = self._steps[0][2]
        result = (self._steps, self._pending)
        if payload == expected:
            result = (self._steps[1:], (-1.0, -1.0, 0))
        elif payload[1:4] == _STATUS and payload[0] == expected[0]:
            self.confirm(payload, moment)
            result = (self._steps, (-1.0, self._pending[1], 0))
        else:
            error = (
                f"Drive {self._number} rejected profile verification "
                f"at {payload[1:4].hex()} with {payload.hex()}"
            )
            raise ValueError(error)
        return result

    @override
    def confirm(self, payload: bytes, moment: float) -> None:
        if int.from_bytes(payload[4:6], "little") & 0x4F in _FAULTS:
            error = (
                f"Drive {self._number} is faulted "
                "and requires explicit recovery"
            )
            raise ValueError(error)
        if moment - self._pending[1] >= _TRANSITION:
            error = f"Drive {self._number} did not reach its requested state"
            raise TimeoutError(error)
