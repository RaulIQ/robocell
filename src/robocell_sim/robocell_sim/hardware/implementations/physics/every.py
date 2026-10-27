"""Report that goes out once per period."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.report import Report


@final
@dataclass(frozen=True)
class Every(Report):
    """Report that lets its origin out only on the boundaries of a period.

    The data holds the live state of the simulation, the period sets in
    seconds of simulated time how often the origin goes out, and the origin
    builds and sends the message. It counts whole physics steps rather than
    seconds, so it keeps no state and never drifts by rounding, and the
    origin is not even built between boundaries.
    """

    _data: mujoco.MjData
    _period: float
    _origin: Report

    @override
    def publish(self) -> None:
        if (
            round(self._data.time / self._data.model.opt.timestep)
            % round(self._period / self._data.model.opt.timestep)
            == 0
        ):
            self._origin.publish()
