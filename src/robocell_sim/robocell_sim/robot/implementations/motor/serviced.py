"""A protective service cycle around the host CAN protocol."""

from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.implementations.motor.listened import Listened
from robocell_sim.robot.implementations.motor.matched import Matched
from robocell_sim.robot.implementations.motor.renewed import Renewed
from robocell_sim.robot.implementations.motor.started import Started
from robocell_sim.robot.implementations.motor.stopped import Stopped
from robocell_sim.robot.interfaces.pilot import Session
from robocell_sim.robot.interfaces.service import Service


@final
@dataclass(frozen=True)
class Serviced(Service):
    """Cycle that supervises both wheels before renewing any nonzero target."""

    _bus: can.BusABC
    _numbers: tuple[int, int]
    _session: Session
    _geometry: tuple[float, float, float, float]

    @override
    def advance(self, moment: float, clock: int) -> Session:
        session = Listened(self._bus, self._numbers, self._session).advance(
            moment, clock
        )
        session = (
            session[0],
            (session[1][0], session[1][1], clock >= 0),
            session[2],
            session[3],
        )
        clock = (
            max(
                (
                    axis.readings()[0][0][4]
                    for axis in session[0]
                    if axis.readings()
                ),
                default=session[2][3],
            )
            if clock < 0
            else clock
        )
        if session[1][0] == "starting":
            session = Serviced(
                self._bus, self._numbers, session, self._geometry
            ).start(moment, clock)
        elif session[1][0] == "enabled":
            Matched(session, self._geometry[3]).verify(moment, clock)
            if Matched(session, self._geometry[3]).disabled():
                Stopped(self._bus, self._numbers).apply(2)
                session = Started(self._numbers, session[0]).session(
                    moment, recovery=False
                )
        return Renewed(
            self._bus, self._numbers, session, self._geometry
        ).advance(moment, clock)

    @override
    def start(self, moment: float, clock: int) -> Session:
        if moment >= self._session[1][1]:
            error = "CANopen startup exceeded its five-second budget"
            raise TimeoutError(error)
        synchronized = all(axis.ready(moment)[2] for axis in self._session[0])
        if synchronized:
            self._bus.send(
                can.Message(
                    arbitration_id=0, data=b"\x01\x00", is_extended_id=False
                ),
                timeout=0.01,
            )
        axes = tuple(
            axis.advance(self._bus, moment, synchronized=synchronized)
            for axis in self._session[0]
        )
        return (
            axes,
            ("enabled", float(clock), self._session[1][2])
            if all(
                axis.ready(moment)[0] and axis.ready(moment)[1] for axis in axes
            )
            else self._session[1],
            (
                self._session[2][0],
                self._session[2][1],
                moment if all(axis.ready(moment)[0] for axis in axes) else -1.0,
                clock,
            ),
            (0.0, 0.0, -1.0),
        )
