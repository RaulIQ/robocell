"""CAN protection around each individual physics boundary."""

import threading
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Serviced(Step):
    """Step that checks drives before motion and acquires after motion."""

    _origin: Step
    _report: Report
    _pause: threading.Event

    @override
    def take(self) -> None:
        self._report.publish()
        if not self._pause.is_set():
            self._origin.take()
        self._report.publish()
