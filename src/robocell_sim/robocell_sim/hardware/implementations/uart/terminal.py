"""A nonblocking operating-system terminal endpoint."""

import os
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port


@final
@dataclass(frozen=True)
class Terminal(Port):
    """Port backed by an already configured, nonblocking descriptor.

    The caller owns the descriptor and keeps the peer open for the session.
    A full terminal buffer accepts no bytes; the caller retains the tail.
    """

    _descriptor: int

    @override
    def receive(self) -> bytes:
        try:
            return os.read(self._descriptor, 4096)
        except BlockingIOError:
            return b""

    @override
    def send(self, payload: bytes) -> int:
        try:
            return os.write(self._descriptor, payload)
        except BlockingIOError:
            return 0
