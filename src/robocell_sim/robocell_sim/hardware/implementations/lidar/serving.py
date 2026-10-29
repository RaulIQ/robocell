"""A serial device serviced by the simulator thread."""

import queue
import time
from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.implementations.uart.queued import Queued
from robocell_sim.hardware.interfaces.device import Device
from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report


@final
@dataclass(frozen=True)
class Serving(Report):
    """Report that advances device acquisition and drains its serial output.

    Data supplies simulated time, the port connects the host, state holds
    the immutable session and pending delivery, and limit bounds backlog.
    Only the simulation thread services this state, including while paused.
    """

    _data: mujoco.MjData
    _port: Port
    _state: queue.SimpleQueue[tuple[Device, bytes, float]]
    _limit: int

    @override
    def publish(self) -> None:
        state = self._state.get_nowait()
        device, payload = state[0].exchange(
            self._port.receive(), self._data.time
        )
        pending = Queued(
            self._port, state[1] + payload, state[2], self._limit
        ).advance(time.monotonic())
        self._state.put((device, *pending))
