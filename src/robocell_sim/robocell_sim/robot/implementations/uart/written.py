"""A complete command sent despite partial serial writes."""

import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report


@final
@dataclass(frozen=True)
class Written(Report):
    """Report that retains an unsent command tail until a bounded deadline."""

    _port: Port
    _payload: bytes
    _timeout: float

    @override
    def publish(self) -> None:
        deadline = time.monotonic() + self._timeout
        pending = self._payload
        while pending and time.monotonic() < deadline:
            pending = pending[self._port.send(pending) :]
            if pending:
                time.sleep(0.001)
        if pending:
            message = "RPLIDAR command write exceeded its deadline"
            raise TimeoutError(message)
