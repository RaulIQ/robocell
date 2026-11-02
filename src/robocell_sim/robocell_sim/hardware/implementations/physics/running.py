"""Catch-up physics that remains protective while paused."""

import threading
from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.physics import Physics
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Running(Physics):
    """Physics that abandons catch-up as soon as a pause is requested."""

    _step: Step
    _data: mujoco.MjData
    _pause: threading.Event
    _report: Report

    @override
    def advance(self, moment: float) -> None:
        self._report.publish()
        while self._data.time < moment and not self._pause.is_set():
            self._step.take()
        self._report.publish()
