"""Operational host heartbeat reception."""

from dataclasses import dataclass
from typing import final, override

import can
import canopen

from robocell_sim.hardware.interfaces.exchange import Exchange
from robocell_sim.hardware.interfaces.session import Session


@final
@dataclass(frozen=True)
class Listened(Exchange):
    """Heartbeat consumer that keeps host liveness separate from commands."""

    _node: canopen.LocalNode
    _session: Session

    @override
    def exchange(self, message: can.Message, moment: float) -> Session:
        motor, (host, sample, number), announced = self._session
        if bytes(message.data) == b"\x05":
            host, motor = moment, motor.event(0, moment)
        elif self._node.nmt.state == "OPERATIONAL":
            motor = motor.event(0x8130, moment)
        return (motor, (host, sample, number), announced)
