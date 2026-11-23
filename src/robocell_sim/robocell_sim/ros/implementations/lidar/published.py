"""A received rotation published as a ROS LaserScan."""

import math
from dataclasses import dataclass
from typing import final, override

from rclpy.time import Time
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.sweep import Sweep
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Published(Report):
    """Report grouping returns on a fixed angular grid."""

    _sweep: Sweep
    _frame: str
    _limits: tuple[float, float]
    _topic: Topic

    @override
    def publish(self) -> None:
        geometry = self._sweep.geometry()
        timing = self._sweep.timing()
        samples = self._sweep.ranges()
        ranges = [math.inf] * 360
        intensities = [0.0] * 360
        for index, (distance, intensity) in enumerate(
            zip(samples, self._sweep.intensities(), strict=True)
        ):
            slot = math.floor(
                math.degrees(geometry[0] + index * geometry[2] + math.pi) + 0.5
            ) % len(ranges)
            if distance < ranges[slot]:
                ranges[slot] = distance
                intensities[slot] = intensity
        self._topic.publish(
            LaserScan(
                header=Header(
                    stamp=Time(
                        nanoseconds=timing[0]
                        + round((len(samples) - 1) * timing[2] * 1e9)
                    ).to_msg(),
                    frame_id=self._frame,
                ),
                angle_min=-math.pi,
                angle_max=math.pi - math.tau / len(ranges),
                angle_increment=math.tau / len(ranges),
                time_increment=0.0,
                scan_time=timing[1],
                range_min=self._limits[0],
                range_max=self._limits[1],
                ranges=ranges,
                intensities=intensities,
            )
        )
