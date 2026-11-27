"""A goal projected toward a target in odometry."""

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import final

from robocell_sim.ros.messages import PoseStamped


@final
@dataclass(frozen=True)
class Goal:
    """Convert a target observation to a stand-off pose facing the target."""

    _approach: Callable[
        [tuple[float, float], tuple[float, float]], tuple[float, float]
    ]

    def __call__(
        self, observation: PoseStamped, robot: tuple[float, float]
    ) -> PoseStamped:
        """Set the navigation goal in the observation's coordinate frame."""
        position = observation.pose.position
        goal = PoseStamped()
        goal.header = observation.header
        goal.pose.position.x, goal.pose.position.y = self._approach(
            (position.x, position.y), robot
        )
        angle = math.atan2(position.y - robot[1], position.x - robot[0])
        goal.pose.orientation.z = math.sin(angle / 2)
        goal.pose.orientation.w = math.cos(angle / 2)
        return goal
