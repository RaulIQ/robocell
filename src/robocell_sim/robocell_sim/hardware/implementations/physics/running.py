"""Catch-up physics that remains protective while paused."""

import threading
import time
from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.physics import Physics
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.hardware.interfaces.step import Step

_BUDGET = 1 / 60


@final
@dataclass(frozen=True)
class Running(Physics):
    """Physics that yields between bounded batches for ROS and the viewer."""

    _step: Step
    _data: mujoco.MjData
    _pause: threading.Event
    _report: Report

    @override
    def advance(self, moment: float) -> None:
        self._report.publish()
        deadline = time.monotonic() + _BUDGET
        while self._data.time < moment and not self._pause.is_set():
            self._step.take()
            if time.monotonic() >= deadline:
                break
        self._report.publish()
