"""SDO requests dispatched to the maintained CANopen library."""

import struct
from dataclasses import dataclass
from typing import final, override

import can
import canopen

from robocell_sim.hardware.implementations.motor.accessible import Accessible
from robocell_sim.hardware.implementations.motor.reported import Reported
from robocell_sim.hardware.interfaces.exchange import Exchange
from robocell_sim.hardware.interfaces.session import Session


@final
@dataclass(frozen=True)
class Requested(Exchange):
    """Request handler that bounds the exposed subset of the library server."""

    _node: canopen.LocalNode
    _session: Session

    @override
    def exchange(self, message: can.Message, moment: float) -> Session:
        if self._node.nmt.state != "STOPPED":
            Reported(self._node, self._session[0]).publish()
            code = Accessible(
                self._node, bytes(message.data), self._session[0].reading()[0]
            ).code()
            if code:
                _, index, subindex = struct.unpack_from("<BHB", message.data)
                self._node.network.send_message(
                    0x580 + self._node.id,
                    struct.pack("<BHBI", 0x80, index, subindex, code),
                )
            else:
                self._node.sdo.on_request(
                    0x600 + self._node.id, message.data, message.timestamp
                )
        return self._session
