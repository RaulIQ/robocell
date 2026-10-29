"""Device output collected while servicing a serial session."""

import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report

from test.reply import Reply


@final
@dataclass(frozen=True)
class Awaited(Reply):
    """Reply with a quiet interval and an independent overall time limit."""

    _report: Report
    _port: Port
    _timeout: float

    @override
    def receive(self) -> bytes:
        deadline = time.monotonic() + self._timeout
        quiet = time.monotonic() + 0.01
        payload = b""
        while time.monotonic() < min(deadline, quiet):
            self._report.publish()
            if chunk := self._port.receive():
                payload += chunk
                quiet = time.monotonic() + 0.01
            time.sleep(0.0005)
        return payload
