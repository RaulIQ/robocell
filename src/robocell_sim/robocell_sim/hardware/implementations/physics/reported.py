"""Step that publishes a report after it."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Reported(Step):
    """Step that moves the world and then publishes a report.

    The origin moves the world by one physics step, and the report goes out
    right after, so it sees the world as that step left it.
    """

    _origin: Step
    _report: Report

    @override
    def take(self) -> None:
        self._origin.take()
        self._report.publish()
