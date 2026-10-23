"""Step that publishes the simulated time."""

from dataclasses import dataclass
from typing import final, override

import mujoco
from rclpy.time import Time
from rosgraph_msgs.msg import Clock  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.step import Step
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Clocked(Step):
    """Step that publishes the time of the simulation after it moves the world.

    The origin moves the world, the data holds the live state of the
    simulation, and the topic carries its time as rosgraph_msgs/Clock, so
    nodes with use_sim_time follow the simulation rather than the wall clock,
    even while it stands paused.
    """

    _origin: Step
    _data: mujoco.MjData
    _topic: Topic

    @override
    def take(self) -> None:
        self._origin.take()
        self._topic.publish(
            Clock(clock=Time(nanoseconds=round(self._data.time * 1e9)).to_msg())
        )
