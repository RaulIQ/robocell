"""Nonblocking expedited SDO and verified drive-state transitions."""

from dataclasses import dataclass, replace
from typing import final, override

import can

from robocell_sim.robot.implementations.motor.answered import Answered
from robocell_sim.robot.implementations.motor.timed import Timed
from robocell_sim.robot.interfaces.negotiation import Exchange, Negotiation

_SDO = 0x600
_SIZE = 8
_OPERATIONAL = 5
_HEARTBEAT = 0.3


@final
@dataclass(frozen=True)
class Negotiated(Negotiation):
    """Handshake whose response waits never block heartbeat or protection.

    Pending holds last transmission, transition start and retry count.
    Heartbeat holds the NMT state and its monotonic reception time.
    """

    _number: int
    _steps: tuple[Exchange, ...]
    _pending: tuple[float, float, int]
    _heartbeat: tuple[int, float]

    @override
    def advance(
        self, bus: can.BusABC, moment: float, *, synchronized: bool
    ) -> Negotiation:
        result = self
        if self.waiting():
            result = (
                replace(self, _steps=self._steps[1:], _pending=(-1.0, -1.0, 0))
                if synchronized
                else self
            )
        elif self._steps:
            steps, pending = Timed(
                bus, self._number, self._steps, self._pending
            ).advance(moment)
            result = replace(self, _steps=steps, _pending=pending)
        return result

    @override
    def read(self, message: can.Message, moment: float) -> Negotiation:
        payload = bytes(message.data)
        result = replace(
            self,
            _heartbeat=(payload[0], moment)
            if message.arbitration_id == 0x700 + self._number
            and len(payload) == 1
            else self._heartbeat,
        )
        if self._steps and self._pending[0] >= 0:
            identifier, request, expected = self._steps[0]
            if (
                not request
                and message.arbitration_id == identifier
                and payload == expected
            ):
                result = replace(
                    result, _steps=self._steps[1:], _pending=(-1.0, -1.0, 0)
                )
            elif (
                identifier == _SDO + self._number
                and message.arbitration_id == 0x580 + self._number
                and len(payload) == _SIZE
                and payload[1:4] == request[1:4]
            ):
                steps, pending = Answered(
                    self._number, self._steps, self._pending
                ).accept(payload, moment)
                result = replace(result, _steps=steps, _pending=pending)
        return result

    @override
    def enabled(self) -> bool:
        return not self._steps

    @override
    def healthy(self, moment: float) -> bool:
        return (
            self._heartbeat[0] == _OPERATIONAL
            and moment - self._heartbeat[1] <= _HEARTBEAT
        )

    @override
    def waiting(self) -> bool:
        return bool(
            self._steps
            and self._steps[0][0] == 0
            and self._steps[0][1] == bytes((1, self._number))
        )
