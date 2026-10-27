"""Journal that warns about lost measurements in a log."""

import logging
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.journal import Journal


@final
@dataclass(frozen=True)
class Logged(Journal):
    """Journal that writes one warning for every frame that lost measurements.

    The log receives the warnings, and a frame that lost nothing writes
    nothing, so a healthy stream keeps the log quiet.
    """

    _log: logging.Logger

    @override
    def lose(self, count: int) -> None:
        if count:
            self._log.warning("IMU lost %d measurements", count)
