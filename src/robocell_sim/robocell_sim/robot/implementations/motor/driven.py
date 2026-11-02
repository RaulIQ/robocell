"""Standalone host lifetime with atomic immutable sessions."""

import math
import queue
from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.implementations.motor.matched import Matched
from robocell_sim.robot.implementations.motor.serviced import Serviced
from robocell_sim.robot.implementations.motor.started import Started
from robocell_sim.robot.implementations.motor.stopped import Stopped
from robocell_sim.robot.interfaces.pilot import Joints, Pilot, Session


@final
@dataclass(frozen=True)
class Driven(Pilot):
    """Host that discards motion on every failure and requires explicit retry.

    Its queue owns one immutable session and is serviced by one caller.
    The geometry contains radius, separation, speed limit and physics step.
    """

    _bus: can.BusABC
    _numbers: tuple[int, int]
    _state: queue.SimpleQueue[Session]
    _geometry: tuple[float, float, float, float]

    @override
    def advance(
        self, moment: float, clock: int
    ) -> tuple[str, tuple[Joints, ...]]:
        session = self._state.get_nowait()
        try:
            session = Serviced(
                self._bus, self._numbers, session, self._geometry
            ).advance(moment, clock)
        except (
            can.CanError,
            OSError,
            ValueError,
            TimeoutError,
            BufferError,
        ) as failure:
            Stopped(self._bus, self._numbers).apply(2)
            session = (
                session[0],
                (f"stopped: {failure}", moment, False),
                session[2],
                (0.0, 0.0, -1.0),
            )
        finally:
            self._state.put(session)
        return (
            session[1][0],
            Matched(session, self._geometry[3]).joints(moment),
        )

    @override
    def command(self, velocity: tuple[float, float], moment: float) -> None:
        if not all(math.isfinite(value) for value in (*velocity, moment)):
            self.close()
            error = "Body command contains a nonfinite velocity or arrival time"
            raise ValueError(error)
        session = self._state.get_nowait()
        self._state.put((*session[:3], (*velocity, moment)))

    @override
    def retry(self, moment: float, *, recovery: bool) -> None:
        Stopped(self._bus, self._numbers).apply(0)
        session = self._state.get_nowait()
        self._state.put(
            Started(self._numbers, session[0]).session(
                moment, recovery=recovery
            )
        )

    @override
    def close(self) -> None:
        session = self._state.get_nowait()
        self._state.put(
            (session[0], ("closed", 0.0, False), session[2], (0.0, 0.0, -1.0))
        )
        Stopped(self._bus, self._numbers).apply(2)
        Stopped(self._bus, self._numbers).apply(0)
