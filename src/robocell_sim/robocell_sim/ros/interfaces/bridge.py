"""Bridge from the simulation to ROS 2."""

import queue
from typing import Protocol

from robocell_sim.ros.interfaces.topic import Topic


class Bridge(Protocol):
    """Bridge that opens the topics between the simulation and ROS 2."""

    def clock(self) -> Topic:
        """Topic of the simulated time, which nodes with use_sim_time follow."""
        ...

    def truth(self) -> Topic:
        """Topic of the true pose and velocity of the robot."""
        ...

    def joints(self) -> Topic:
        """Topic of the wheel angles, which robot_state_publisher reads."""
        ...

    def frames(self) -> Topic:
        """Topic of the moving transforms between the frames of the robot."""
        ...

    def listen(self, inbox: queue.SimpleQueue[tuple[float, float]]) -> None:
        """Queue every velocity command as forward speed and yaw rate."""
        ...
