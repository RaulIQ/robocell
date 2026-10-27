"""Report that publishes the simulated time."""

from dataclasses import dataclass
from typing import final, override

import mujoco
from rclpy.time import Time
from rosgraph_msgs.msg import Clock  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Clocked(Report):
    """Report that publishes the time of the simulation.

    The data holds the live state of the simulation, and the topic carries
    its time as rosgraph_msgs/Clock, so nodes with use_sim_time follow the
    simulation rather than the wall clock, even while it stands paused.
    """

    _data: mujoco.MjData
    _topic: Topic

    @override
    def publish(self) -> None:
        self._topic.publish(
            Clock(clock=Time(nanoseconds=round(self._data.time * 1e9)).to_msg())
        )
