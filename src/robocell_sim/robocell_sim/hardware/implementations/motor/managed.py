"""NMT communication state changes with immediate output inhibition."""

from dataclasses import dataclass
from typing import final, override

import can
import canopen

from robocell_sim.hardware.implementations.motor.restarted import Restarted
from robocell_sim.hardware.interfaces.exchange import Exchange
from robocell_sim.hardware.interfaces.session import Session

_COMMANDS = {1, 2, 128, 129, 130}
_RESETS = {129, 130}


@final
@dataclass(frozen=True)
class Managed(Exchange):
    """NMT handler that never interprets communication start as drive enable."""

    _node: canopen.LocalNode
    _session: Session

    @override
    def exchange(self, message: can.Message, moment: float) -> Session:
        command, number = message.data
        session = self._session
        if number not in {0, self._node.id} or command not in _COMMANDS:
            return session
        if command in _RESETS:
            session = Restarted(self._node, session).reset(command, moment)
        else:
            motor = (
                session[0].event(0, moment)
                if command == 1 and self._node.nmt.state != "OPERATIONAL"
                else session[0]
                if command == 1
                else session[0].event(-1, moment)
            )
            self._node.nmt.on_command(0, message.data, message.timestamp)
            session = (motor, session[1], session[2])
        return session
