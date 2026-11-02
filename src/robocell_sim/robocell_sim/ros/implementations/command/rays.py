"""Arbitrary LaserScan bearings mapped into the guard's degree bins."""

import math
from dataclasses import dataclass
from typing import final, override

from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.ranges import Ranges


@final
@dataclass(frozen=True)
class Rays(Ranges):
    """Ranges keeping the closest valid return when bearings share a bin."""

    _message: LaserScan

    @override
    def ranges(self) -> tuple[float, ...]:
        if (
            not self._message.ranges
            or not all(
                math.isfinite(value)
                for value in (
                    self._message.angle_min,
                    self._message.angle_increment,
                )
            )
            or self._message.angle_increment == 0
        ):
            error = "Collision guard received an empty scan or invalid bearings"
            raise ValueError(error)
        ranges = [math.inf] * 360
        for offset, distance in enumerate(self._message.ranges):
            if (
                math.isfinite(distance)
                and self._message.range_min
                <= distance
                <= self._message.range_max
            ):
                index = round(
                    math.degrees(
                        self._message.angle_min
                        + offset * self._message.angle_increment
                    )
                ) % len(ranges)
                ranges[index] = min(ranges[index], distance)
        return tuple(ranges)
