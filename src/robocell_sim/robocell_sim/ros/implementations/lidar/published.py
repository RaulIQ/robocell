"""A received rotation published as a ROS LaserScan."""

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
    """Report preserving the received scan's angular and temporal order."""

    _sweep: Sweep
    _frame: str
    _limits: tuple[float, float]
    _topic: Topic

    @override
    def publish(self) -> None:
        geometry = self._sweep.geometry()
        timing = self._sweep.timing()
        self._topic.publish(
            LaserScan(
                header=Header(
                    stamp=Time(nanoseconds=timing[0]).to_msg(),
                    frame_id=self._frame,
                ),
                angle_min=geometry[0],
                angle_max=geometry[1],
                angle_increment=geometry[2],
                time_increment=timing[2],
                scan_time=timing[1],
                range_min=self._limits[0],
                range_max=self._limits[1],
                ranges=list(self._sweep.ranges()),
                intensities=list(self._sweep.intensities()),
            )
        )
