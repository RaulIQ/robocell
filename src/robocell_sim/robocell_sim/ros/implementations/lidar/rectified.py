"""An instantaneous, fixed angular grid for the SLAM consumer."""

import math
from dataclasses import dataclass
from typing import final, override

from rclpy.time import Time
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.ros.implementations.lidar.compensated import Compensated
from robocell_sim.ros.interfaces.coordinates import Coordinates
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Rectified(Report):
    """Report of deskewed returns with nearest-obstacle collision selection."""

    _scan: LaserScan
    _coordinates: Coordinates
    _count: int
    _topic: Topic

    @override
    def publish(self) -> None:
        if not all(
            (
                self._count > 1,
                bool(self._scan.ranges),
                bool(self._scan.header.frame_id),
                Time.from_msg(self._scan.header.stamp).nanoseconds > 0,
                math.isfinite(self._scan.angle_min),
                math.isfinite(self._scan.angle_increment),
                self._scan.angle_increment != 0,
                math.isfinite(self._scan.time_increment),
                self._scan.time_increment >= 0,
                math.isfinite(self._scan.scan_time),
                self._scan.scan_time >= 0,
                0 < self._scan.range_min < self._scan.range_max < math.inf,
                len(self._scan.intensities) in {0, len(self._scan.ranges)},
            )
        ):
            message = "SLAM scan has invalid geometry or acquisition metadata"
            raise ValueError(message)
        reference = Time.from_msg(self._scan.header.stamp).nanoseconds + round(
            (len(self._scan.ranges) - 1) * self._scan.time_increment * 1e9
        )
        bins = Compensated(
            self._scan, self._coordinates, self._count, reference
        ).returns()
        self._topic.publish(
            LaserScan(
                header=Header(
                    stamp=Time(nanoseconds=reference).to_msg(),
                    frame_id=self._scan.header.frame_id,
                ),
                angle_min=-math.pi,
                angle_max=-math.pi + (self._count - 1) * math.tau / self._count,
                angle_increment=math.tau / self._count,
                time_increment=0.0,
                scan_time=self._scan.scan_time,
                range_min=self._scan.range_min,
                range_max=self._scan.range_max,
                ranges=[
                    bins.get(index, (math.inf, 0.0))[0]
                    for index in range(self._count)
                ],
                intensities=[
                    bins.get(index, (math.inf, 0.0))[1]
                    for index in range(self._count)
                ],
            )
        )
