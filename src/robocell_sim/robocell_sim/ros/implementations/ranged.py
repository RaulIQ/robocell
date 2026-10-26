"""Report that publishes the scan of the lidar."""

import math
from dataclasses import dataclass
from typing import final, override

import mujoco
from rclpy.time import Time
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.lidar import Lidar
from robocell_sim.ros.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Ranged(Report):
    """Report that publishes what the lidar sees around the robot.

    The data holds the live state of the simulation, the lidar ranges the
    world in a level plane, the limits are the shortest and the longest
    range in meters it reports, and the topic carries the scan as
    sensor_msgs/LaserScan in the base_scan frame of the URDF, stamped with
    the simulated time. The rays split the full turn evenly and run
    counterclockwise from ahead, as ROS expects, with inf for no echo.
    """

    _data: mujoco.MjData
    _lidar: Lidar
    _limits: tuple[float, float]
    _topic: Topic

    @override
    def publish(self) -> None:
        ranges = self._lidar.ranges()
        self._topic.publish(
            LaserScan(
                header=Header(
                    stamp=Time(
                        nanoseconds=round(self._data.time * 1e9)
                    ).to_msg(),
                    frame_id="base_scan",
                ),
                angle_min=0.0,
                angle_max=2 * math.pi * (len(ranges) - 1) / len(ranges),
                angle_increment=2 * math.pi / len(ranges),
                range_min=self._limits[0],
                range_max=self._limits[1],
                ranges=list(ranges),
            )
        )
