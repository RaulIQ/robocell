"""Inbox that drains a python-can bus."""

from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.interfaces.inbox import Inbox


@final
@dataclass(frozen=True)
class Drained(Inbox):
    """Inbox that takes every frame waiting in the buffer of a bus.

    The origin is an open python-can bus, SocketCAN on vcan0 in the driver
    or a virtual one in tests, and the filters it was opened with decide
    which identifiers reach the buffer at all. Each call empties the
    buffer and never waits for a frame that has not arrived yet.
    """

    _origin: can.BusABC

    @override
    def frames(self) -> tuple[tuple[int, bytes], ...]:
        return tuple(
            (message.arbitration_id, bytes(message.data))
            for message in iter(lambda: self._origin.recv(0), None)
        )
