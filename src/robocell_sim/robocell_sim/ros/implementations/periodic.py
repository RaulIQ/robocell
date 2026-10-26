"""Step that publishes a report once per period."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.step import Step
from robocell_sim.ros.interfaces.report import Report


@final
@dataclass(frozen=True)
class Periodic(Step):
    """Step that publishes a report only on the boundaries of a period.

    The origin moves the world, the data holds the live state of the
    simulation, the period sets in seconds of simulated time how often the
    report goes out, and the report builds and sends the message. It counts
    whole physics steps rather than seconds, so it keeps no state and never
    drifts by rounding, and the report is not even built between boundaries.
    """

    _origin: Step
    _data: mujoco.MjData
    _period: float
    _report: Report

    @override
    def take(self) -> None:
        self._origin.take()
        if (
            round(self._data.time / self._data.model.opt.timestep)
            % round(self._period / self._data.model.opt.timestep)
            == 0
        ):
            self._report.publish()
