"""An isolated CAN host against independent wheel emulators."""

import queue
import threading
import time
from collections.abc import Generator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING, final, override

import can
import mujoco
from robocell_sim.robot.implementations.motor.driven import Driven
from robocell_sim.robot.implementations.motor.started import Started
from robocell_sim.robot.interfaces.pilot import Pilot

from test.flight import Flight
from test.flown import Flown
from test.journey import Journey
from test.motored import Motored

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.pilot import Session


@final
@dataclass(frozen=True)
class Piloted(Journey):
    """Fresh CAN channel and physical shafts for every host experiment."""

    _numbers: tuple[int, int]
    _external: bool

    @override
    @contextmanager
    def connection(
        self,
    ) -> Generator[
        tuple[Flight, Pilot, mujoco.MjData, can.BusABC, threading.Event]
    ]:
        with Motored(self._numbers).connection() as (bus, devices, data, pause):
            state: queue.SimpleQueue[Session] = queue.SimpleQueue()
            state.put(
                Started(self._numbers, ()).session(
                    time.monotonic(), recovery=False
                )
            )
            pilot = Driven(
                bus, self._numbers, state, (0.033, 0.288, 7.88, 0.002)
            )
            try:
                yield (
                    Flown(pilot, devices, data, _external=self._external),
                    pilot,
                    data,
                    bus,
                    pause,
                )
            finally:
                pilot.close()
