"""A serial watchdog aware of a paused ROS clock."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.activity import Activity


@final
@dataclass(frozen=True)
class Watched(Activity):
    """Activity that times out missing data only while ROS time progresses."""

    _activity: tuple[float, int, float]
    _timeout: float

    @override
    def read(
        self, payload: bytes, stamp: int, moment: float
    ) -> tuple[float, int, float]:
        if stamp < self._activity[1]:
            message = (
                "RPLIDAR ROS clock moved backwards, so restart acquisition"
            )
            raise ValueError(message)
        if (
            stamp != self._activity[1]
            and moment - self._activity[2] >= self._timeout / 2
        ):
            return (moment, stamp, moment)
        if (
            not payload
            and moment - self._activity[0] > self._timeout
            and moment - self._activity[2] < self._timeout / 2
        ):
            message = "RPLIDAR measurements timed out while ROS time advanced"
            raise TimeoutError(message)
        return (
            moment if payload else self._activity[0],
            stamp,
            moment if stamp != self._activity[1] else self._activity[2],
        )
