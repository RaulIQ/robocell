"""Incremental standard-SCAN packet boundaries."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.lidar.decoded import Decoded
from robocell_sim.robot.interfaces.samples import Samples

_SIZE = 5


@final
@dataclass(frozen=True)
class Parsed(Samples):
    """Samples that move one byte after damage and preserve a partial tail."""

    _payload: bytes

    @override
    def split(
        self,
    ) -> tuple[tuple[tuple[bool, bool, float, float, int], ...], bytes]:
        pending = self._payload
        events: list[tuple[bool, bool, float, float, int]] = []
        while len(pending) >= _SIZE:
            echo = Decoded(pending[:_SIZE])
            events.append(
                (True, *echo.values())
                if echo.valid()
                else (False, False, 0.0, 0.0, 0)
            )
            pending = pending[_SIZE if echo.valid() else 1 :]
        return (tuple(events), pending)
