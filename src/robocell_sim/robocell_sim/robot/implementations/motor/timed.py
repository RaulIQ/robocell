"""Bounded nonblocking retransmission of an initialization exchange."""

from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.interfaces.negotiation import Exchange
from robocell_sim.robot.interfaces.transmission import Progress, Transmission

_TIMEOUT = 0.25
_RETRIES = 2


@final
@dataclass(frozen=True)
class Timed(Transmission):
    """Request whose wall-time limit never suspends host supervision."""

    _bus: can.BusABC
    _number: int
    _steps: tuple[Exchange, ...]
    _pending: tuple[float, float, int]

    @override
    def advance(self, moment: float) -> Progress:
        identifier, request, expected = self._steps[0]
        sent, _, _ = self._pending
        result = (self._steps, self._pending)
        if request and not expected:
            self._bus.send(
                can.Message(
                    arbitration_id=identifier,
                    data=request,
                    is_extended_id=False,
                ),
                timeout=0.01,
            )
            result = (self._steps[1:], (-1.0, -1.0, 0))
        elif sent < 0 or moment - sent >= _TIMEOUT:
            result = self.renew(moment)
        return result

    @override
    def renew(self, moment: float) -> Progress:
        identifier, request, _ = self._steps[0]
        _, beginning, retries = self._pending
        if request and retries > _RETRIES:
            error = f"Drive {self._number} did not answer its startup request"
            raise TimeoutError(error)
        if request:
            self._bus.send(
                can.Message(
                    arbitration_id=identifier,
                    data=request,
                    is_extended_id=False,
                ),
                timeout=0.01,
            )
        return (
            self._steps,
            (
                moment,
                moment if beginning < 0 else beginning,
                retries + bool(request),
            ),
        )
