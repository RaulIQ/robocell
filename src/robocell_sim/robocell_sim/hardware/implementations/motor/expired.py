"""Independent host, command and stopping deadlines."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.access import Access

_ENABLED = 0x27
_QUICK = 7
_STOPPED = 0.01


@final
@dataclass(frozen=True)
class Expired(Access):
    """Deadline decision that reports the first currently expired protection."""

    _status: int
    _history: tuple[int, float, float, float]
    _moment: float
    _limits: tuple[float, float, float]

    @override
    def code(self) -> int:
        return next(
            (
                code
                for expired, code in (
                    (
                        self._moment - self._history[2] >= self._limits[1],
                        0x8130,
                    ),
                    (
                        self._status == _ENABLED
                        and self._moment - self._history[1] >= self._limits[0],
                        0xFF01,
                    ),
                    (
                        self._status == _QUICK
                        and abs(self._limits[2]) >= _STOPPED
                        and self._moment - self._history[3] >= 1.0,
                        0xFF02,
                    ),
                )
                if expired
            ),
            0,
        )
