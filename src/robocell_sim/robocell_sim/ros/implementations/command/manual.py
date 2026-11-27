"""Publication of the selected manual drive command."""

from dataclasses import dataclass
from typing import final

from geometry_msgs.msg import Twist  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.context import Context

from robocell_sim.ros.interfaces.topic import Topic
from robocell_sim.ros.messages import Vector3

__all__ = ["Manual", "Twist"]


@final
@dataclass(frozen=True)
class Manual:
    """Send body velocity to the currently selected robot or cube."""

    _context: Context
    _destination: list[Topic]

    def __call__(self, velocity: tuple[float, float]) -> None:
        """Send the held velocity while the ROS node is running."""
        if self._context.ok():
            self._destination[0].publish(
                Twist(
                    linear=Vector3(x=velocity[0]),
                    angular=Vector3(z=velocity[1]),
                )
            )
