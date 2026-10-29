"""Validated standard sample timing of an A1."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.cadence import Cadence


@final
@dataclass(frozen=True)
class Sampling(Cadence):
    """Cadence extracted from the standard half of GET_SAMPLERATE."""

    _payload: bytes

    @override
    def period(self) -> int:
        period = int.from_bytes(self._payload[:2], "little")
        if not period:
            message = "RPLIDAR reported a zero standard sample period"
            raise ValueError(message)
        return period
