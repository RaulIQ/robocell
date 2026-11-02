"""Classic CAN frame routing for one local drive."""

from dataclasses import dataclass
from typing import final, override

import can
import canopen

from robocell_sim.hardware.implementations.motor.commanded import Commanded
from robocell_sim.hardware.implementations.motor.listened import Listened
from robocell_sim.hardware.implementations.motor.managed import Managed
from robocell_sim.hardware.implementations.motor.requested import Requested
from robocell_sim.hardware.interfaces.exchange import Exchange
from robocell_sim.hardware.interfaces.session import Session
from robocell_sim.hardware.interfaces.shaft import Shaft


@final
@dataclass(frozen=True)
class Routed(Exchange):
    """Routing that rejects unsupported frame flags and malformed lengths."""

    _node: canopen.LocalNode
    _shaft: Shaft
    _session: Session

    @override
    def exchange(self, message: can.Message, moment: float) -> Session:
        routes: dict[int, tuple[int, Exchange]] = {
            0: (2, Managed(self._node, self._session)),
            0x77F: (1, Listened(self._node, self._session)),
            0x200 + self._node.id: (
                6,
                Commanded(self._node, self._shaft, self._session),
            ),
            0x600 + self._node.id: (8, Requested(self._node, self._session)),
        }
        session = self._session
        if (
            not any(
                (
                    message.is_extended_id,
                    message.is_remote_frame,
                    message.is_error_frame,
                    message.is_fd,
                )
            )
            and message.arbitration_id in routes
        ):
            length, route = routes[message.arbitration_id]
            if message.dlc == len(message.data) == length:
                session = route.exchange(message, moment)
        if self._shaft.paused():
            session = (session[0].event(-1, moment), session[1], session[2])
        self._shaft.spin(session[0].reading()[1] / 1000)
        return session
