"""Report that notes when it was published."""

from dataclasses import dataclass
from typing import final, override

import mujoco
from robocell_sim.ros.interfaces.report import Report


@final
@dataclass(frozen=True)
class Noted(Report):
    """Report that appends the simulated time to a list instead of sending.

    The list receives the time of every publication in order, and the data
    holds the live state of the simulation whose clock it reads, so a test
    sees when a ROS 2 subscriber would have heard from the simulation.
    """

    _times: list[float]
    _data: mujoco.MjData

    @override
    def publish(self) -> None:
        self._times.append(self._data.time)
