"""Classic CANopen velocity drive serviced by the physics thread."""

import queue
from dataclasses import dataclass
from typing import final, override

import can
import canopen

from robocell_sim.hardware.implementations.motor.protected import Protected
from robocell_sim.hardware.implementations.motor.published import Published
from robocell_sim.hardware.implementations.motor.received import Received
from robocell_sim.hardware.interfaces.drive import Drive
from robocell_sim.hardware.interfaces.session import Session
from robocell_sim.hardware.interfaces.shaft import Shaft


@final
@dataclass(frozen=True)
class Emulated(Drive):
    """Endpoint that composes protocol service, protection and acquisition.

    The queue owns the immutable session, and only the physics thread
    services the endpoint or accesses its shaft.
    """

    _bus: can.BusABC
    _node: canopen.LocalNode
    _shaft: Shaft
    _state: queue.SimpleQueue[Session]

    @override
    def advance(self, moment: float) -> None:
        session = self._state.get_nowait()
        try:
            session = Published(
                self._node,
                self._shaft,
                Received(
                    self._bus,
                    self._node,
                    self._shaft,
                    Protected(self._node, self._shaft, session).advance(moment),
                ).advance(moment),
            ).advance(moment)
        except BaseException:
            self._shaft.spin(0.0)
            session = (session[0].event(0xFF02, moment), session[1], session[2])
            raise
        finally:
            self._state.put(session)

    @override
    def close(self) -> None:
        self._shaft.spin(0.0)
        self._node.nmt.stop_heartbeat()
        self._node.remove_network()
