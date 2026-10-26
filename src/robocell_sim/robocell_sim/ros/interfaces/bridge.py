"""Bridge from the simulation to ROS 2."""

from typing import Protocol

from robocell_sim.ros.interfaces.topic import Topic


class Bridge(Protocol):
    """Bridge that opens the topics only the simulation can offer ROS 2.

    The real robot has no such topics, so they go away on the hardware.
    """

    def clock(self) -> Topic:
        """Topic of the simulated time, which nodes with use_sim_time follow."""
        ...

    def truth(self) -> Topic:
        """Topic of the true pose and velocity of the robot."""
        ...

    def frames(self) -> Topic:
        """Topic of the moving transforms between the frames of the robot."""
        ...
