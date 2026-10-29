"""The nanosecond clock of a ROS node."""

from dataclasses import dataclass
from typing import final, override

from rclpy.node import Node

from robocell_sim.robot.interfaces.clock import Clock


@final
@dataclass(frozen=True)
class Timed(Clock):
    """Clock that follows the node's configured ROS time source."""

    _node: Node

    @override
    def now(self) -> int:
        return self._node.get_clock().now().nanoseconds
