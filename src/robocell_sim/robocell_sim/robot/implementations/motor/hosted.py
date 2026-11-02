"""Validated transport and lifetime of a pair of wheel drives."""

import math
import queue
from collections.abc import Generator
from contextlib import closing, contextmanager
from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.implementations.motor.driven import Driven
from robocell_sim.robot.implementations.motor.started import Started
from robocell_sim.robot.interfaces.hosting import Hosting
from robocell_sim.robot.interfaces.pilot import Pilot, Session

_LIMIT = 7.88


@final
@dataclass(frozen=True)
class Hosted(Hosting):
    """Connection preserving accumulated travel across host retries."""

    _transport: tuple[str, str]
    _numbers: tuple[int, int]
    _geometry: tuple[float, float, float, float]
    _state: queue.SimpleQueue[Session]

    @override
    @contextmanager
    def connection(self, moment: float, *, recovery: bool) -> Generator[Pilot]:
        if (
            not all(
                math.isfinite(value) and value > 0 for value in self._geometry
            )
            or self._geometry[2] > _LIMIT
        ):
            error = (
                "Drive geometry must be finite and positive "
                "with a limit at most 7.88 rad/s"
            )
            raise ValueError(error)
        self._state.put(
            Started(
                self._numbers,
                self._state.get_nowait()[0] if not self._state.empty() else (),
            ).session(moment, recovery=recovery)
        )
        with (
            can.Bus(
                interface=self._transport[0], channel=self._transport[1]
            ) as bus,
            closing(
                Driven(bus, self._numbers, self._state, self._geometry)
            ) as pilot,
        ):
            yield pilot
