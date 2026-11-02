"""Lifetime of a CANopen node and its MuJoCo shaft."""

import queue
import time
from collections.abc import Generator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import TYPE_CHECKING, final, override

import can
import canopen

from robocell_sim.hardware.implementations.motor.booted import Booted
from robocell_sim.hardware.implementations.motor.controlled import Controlled
from robocell_sim.hardware.implementations.motor.described import Described
from robocell_sim.hardware.implementations.motor.emulated import Emulated
from robocell_sim.hardware.interfaces.attachment import Attachment
from robocell_sim.hardware.interfaces.drive import Drive

if TYPE_CHECKING:
    from robocell_sim.hardware.interfaces.session import Session
from robocell_sim.hardware.interfaces.shaft import Shaft


@final
@dataclass(frozen=True)
class Attached(Attachment):
    """Attachment that creates an independent CANopen endpoint for one shaft."""

    _bus: can.BusABC
    _number: int
    _shaft: Shaft

    @override
    @contextmanager
    def connection(self) -> Generator[Drive]:
        node = canopen.Network(self._bus).add_node(
            Described(self._number).node()
        )
        state: queue.SimpleQueue[Session] = queue.SimpleQueue()
        state.put(
            (
                Controlled(
                    0x40, 0, 0, (0, time.monotonic(), time.monotonic(), 0.0)
                ),
                (-float("inf"), -1.0, 0),
                0,
            )
        )
        device = Emulated(self._bus, node, self._shaft, state)
        try:
            Booted(node, self._shaft).publish()
            yield device
        finally:
            device.close()
