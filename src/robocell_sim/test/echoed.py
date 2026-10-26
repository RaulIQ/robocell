"""Lidar that hears the same echo on every ray."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.lidar import Lidar


@final
@dataclass(frozen=True)
class Echoed(Lidar):
    """Lidar whose every ray comes back from the same distance.

    The echo is that distance in meters, or inf for no echo at all, and the
    count is how many rays split the full turn, so a test sets the shape of
    a scan without a world around the robot.
    """

    _echo: float
    _count: int

    @override
    def ranges(self) -> tuple[float, ...]:
        return (self._echo,) * self._count
