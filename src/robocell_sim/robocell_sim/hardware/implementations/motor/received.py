"""Bounded reception on one drive's own CAN endpoint."""

from dataclasses import dataclass
from typing import final, override

import can
import canopen

from robocell_sim.hardware.implementations.motor.routed import Routed
from robocell_sim.hardware.interfaces.progression import Progression
from robocell_sim.hardware.interfaces.session import Session
from robocell_sim.hardware.interfaces.shaft import Shaft

_LIMIT = 256


@final
@dataclass(frozen=True)
class Received(Progression):
    """Reception that cannot starve protection through an unbounded backlog."""

    _bus: can.BusABC
    _node: canopen.LocalNode
    _shaft: Shaft
    _session: Session

    @override
    def advance(self, moment: float) -> Session:
        session = self._session
        for _ in range(_LIMIT):
            message = self._bus.recv(0)
            if message is None:
                break
            session = Routed(self._node, self._shaft, session).exchange(
                message, moment
            )
        else:
            error = "Drive CAN service exhausted its 256-frame budget"
            raise BufferError(error)
        self._shaft.spin(session[0].reading()[1] / 1000)
        return session
