"""Lidar points placed in room axes by a pose."""

import math
from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.cloud import Cloud


@final
@dataclass(frozen=True)
class Placed(Cloud):
    """Cloud that puts the echoes of one scan around a pose of the robot.

    The scan holds ranges in meters, one per degree counterclockwise from
    ahead, with infinity where nothing echoed. The pose holds x and y in
    meters and the heading in radians, and the mount is how far ahead of
    that pose the scanner sits, negative when it sits behind.
    """

    _scan: tuple[float, ...]
    _pose: tuple[float, float, float]
    _mount: float

    @override
    def points(self) -> list[tuple[float, float]]:
        return [
            (
                self._pose[0]
                + self._mount * math.cos(self._pose[2])
                + distance * math.cos(self._pose[2] + math.radians(index)),
                self._pose[1]
                + self._mount * math.sin(self._pose[2])
                + distance * math.sin(self._pose[2] + math.radians(index)),
            )
            for index, distance in enumerate(self._scan)
            if math.isfinite(distance)
        ]
