"""A waypoint separated from the target's occupied cell."""

from dataclasses import dataclass
from math import dist, isfinite
from typing import final


@final
@dataclass(frozen=True)
class Approached:
    """Choose a waypoint at the configured stand-off distance."""

    _distance: float

    def __call__(
        self, target: tuple[float, float], robot: tuple[float, float]
    ) -> tuple[float, float]:
        """Choose a point in front of the target or stay at the robot pose."""
        if (
            not all(
                isfinite(value) for value in (*target, *robot, self._distance)
            )
            or self._distance <= 0
        ):
            error = (
                "Approach requires finite coordinates and a positive distance"
            )
            raise ValueError(error)
        separation = dist(target, robot)
        if separation <= self._distance:
            return robot
        return (
            target[0] + (robot[0] - target[0]) * self._distance / separation,
            target[1] + (robot[1] - target[1]) * self._distance / separation,
        )
