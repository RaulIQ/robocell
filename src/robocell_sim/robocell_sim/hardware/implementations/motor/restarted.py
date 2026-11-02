"""Volatile CANopen resets without moving the physical encoder origin."""

import math
from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.implementations.motor.controlled import Controlled
from robocell_sim.hardware.interfaces.reset import Reset
from robocell_sim.hardware.interfaces.session import Session

_COMMUNICATION = (0x1000, 0x2000)
_NODE = 129
_LOCAL = 0xFF02


@final
@dataclass(frozen=True)
class Restarted(Reset):
    """Reset that discards motion and restores the selected volatile objects."""

    _node: canopen.LocalNode
    _session: Session

    @override
    def reset(self, command: int, moment: float) -> Session:
        motor = self._session[0].event(-1, moment)
        for index in tuple(self._node.data_store):
            if (
                command == _NODE
                or _COMMUNICATION[0] <= index < _COMMUNICATION[1]
            ):
                del self._node.data_store[index]
        if all(
            (
                command == _NODE,
                motor.reading()[2] != _LOCAL,
                moment - self._session[1][0]
                < (
                    int.from_bytes(self._node.get_data(0x1016, 1), "little")
                    & 0xFFFF
                )
                / 1000,
            )
        ):
            motor = Controlled(0x40, 0, 0, (0, moment, moment, 0.0))
        self._node.nmt.send_command(command)
        self._node.nmt.send_command(128)
        return (motor, (-math.inf, -1.0, 0), self._session[2])
