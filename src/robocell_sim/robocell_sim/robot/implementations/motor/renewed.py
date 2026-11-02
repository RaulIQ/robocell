"""Monotonic host heartbeat and independently aged cyclic commands."""

import math
import struct
from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.drive.limited import Limited
from robocell_sim.robot.interfaces.pilot import Session
from robocell_sim.robot.interfaces.renewal import Renewal

_HEARTBEAT = 0.1
_PERIOD = 0.02
_COMMAND = 0.25


@final
@dataclass(frozen=True)
class Renewed(Renewal):
    """Renewal that keeps command age independent from PDO transmission."""

    _bus: can.BusABC
    _numbers: tuple[int, int]
    _session: Session
    _geometry: tuple[float, float, float, float]

    @override
    def advance(self, moment: float, clock: int) -> Session:
        heartbeat, sent, enabled, _ = self._session[2]
        if moment - heartbeat >= _HEARTBEAT or heartbeat < 0:
            self._bus.send(
                can.Message(
                    arbitration_id=0x77F, data=b"\x05", is_extended_id=False
                ),
                timeout=0.01,
            )
            heartbeat = moment
        if moment - sent >= _PERIOD or sent < 0:
            self.transmit(moment)
            sent = moment
        return (
            self._session[0],
            self._session[1],
            (heartbeat, sent, enabled, clock),
            self._session[3],
        )

    @override
    def transmit(self, moment: float) -> None:
        targets = (
            Limited(
                Differential(self._geometry[0], self._geometry[1]),
                self._geometry[2],
            ).wheels(*self._session[3][:2])
            if self._session[1][0] == "enabled"
            and self._session[3][2] > self._session[2][2]
            and moment - self._session[3][2] <= _COMMAND
            else (0.0, 0.0)
        )
        for number, axis, target in zip(
            self._numbers, self._session[0], targets, strict=True
        ):
            if axis.ready(moment)[0] and self._session[1][0] in {
                "starting",
                "enabled",
            }:
                self._bus.send(
                    can.Message(
                        arbitration_id=0x200 + number,
                        data=struct.pack(
                            "<Hi",
                            15,
                            math.floor(target * 1000 + 0.5)
                            if target >= 0
                            else math.ceil(target * 1000 - 0.5),
                        ),
                        is_extended_id=False,
                    ),
                    timeout=0.01,
                )
