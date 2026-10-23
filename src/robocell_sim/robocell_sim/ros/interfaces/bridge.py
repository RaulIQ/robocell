"""Bridge from the simulation to ROS 2."""

from typing import Protocol

from robocell_sim.ros.interfaces.topic import Topic


class Bridge(Protocol):
    """Bridge that opens the topics the simulation publishes to ROS 2."""

    def clock(self) -> Topic:
        """Topic of the simulated time, which nodes with use_sim_time follow."""

    def truth(self) -> Topic:
        """Topic of the true pose and velocity of the robot."""
