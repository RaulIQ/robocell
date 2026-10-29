"""Stale bytes drained before a new command session."""

import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report


@final
@dataclass(frozen=True)
class Flushed(Report):
    """Report that waits for a short quiet interval under an overall bound."""

    _port: Port
    _timeout: float

    @override
    def publish(self) -> None:
        deadline = time.monotonic() + self._timeout
        quiet = time.monotonic() + 0.02
        while time.monotonic() < min(deadline, quiet):
            if self._port.receive():
                quiet = time.monotonic() + 0.02
            time.sleep(0.001)
        if time.monotonic() >= deadline:
            message = "RPLIDAR input did not become quiet before the deadline"
            raise TimeoutError(message)
