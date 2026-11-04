"""Nearest-return binning of motion-compensated scan points."""

import math
from dataclasses import dataclass
from typing import final, override

from rclpy.time import Time
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.implementations.lidar.projected import Projected
from robocell_sim.ros.interfaces.coordinates import Coordinates
from robocell_sim.ros.interfaces.grid import Grid


@final
@dataclass(frozen=True)
class Compensated(Grid):
    """Fixed bins retaining the nearest valid projected return."""

    _scan: LaserScan
    _coordinates: Coordinates
    _count: int
    _reference: int

    @override
    def returns(self) -> dict[int, tuple[float, float]]:
        self._coordinates.point(
            (0.0, 0.0),
            Time.from_msg(self._scan.header.stamp).nanoseconds,
            self._reference,
        )
        self._coordinates.point((0.0, 0.0), self._reference, self._reference)
        bins: dict[int, tuple[float, float]] = {}
        for index, distance in enumerate(self._scan.ranges):
            if (
                not math.isfinite(distance)
                or not self._scan.range_min <= distance <= self._scan.range_max
            ):
                continue
            radius, angle, intensity = Projected(
                (
                    distance,
                    self._scan.angle_min + index * self._scan.angle_increment,
                    self._scan.intensities[index]
                    if self._scan.intensities
                    else 0.0,
                ),
                self._coordinates,
                (
                    Time.from_msg(self._scan.header.stamp).nanoseconds
                    + round(index * self._scan.time_increment * 1e9),
                    self._reference,
                ),
                (self._scan.range_min, self._scan.range_max),
            ).reading()
            slot = (
                math.floor(
                    ((angle + math.pi) % math.tau) * self._count / math.tau
                    + 0.5
                )
                % self._count
            )
            if radius < bins.get(slot, (math.inf, 0.0))[0]:
                bins[slot] = (radius, intensity)
        return bins
