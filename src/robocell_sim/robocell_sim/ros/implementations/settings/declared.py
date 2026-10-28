"""Settings that a ROS 2 node declares as its parameters."""

from dataclasses import dataclass
from typing import final, override

from rclpy.node import Node

from robocell_sim.ros.interfaces.settings import Settings


@final
@dataclass(frozen=True)
class Declared(Settings):
    """Settings that live as declared parameters of one node.

    The node declares each parameter on the first read, so the launch
    overrides it and ros2 param lists it. The type of the default fixes
    the type of the parameter, so a launch that sets another type fails.
    rclpy leaves parameter values untyped, so every value passes through
    its text, which keeps each number exact.
    """

    _node: Node

    @override
    def number(self, name: str, default: float) -> float:
        return float(str(self._node.declare_parameter(name, default).value))

    @override
    def whole(self, name: str, default: int) -> int:
        return int(str(self._node.declare_parameter(name, default).value))

    @override
    def text(self, name: str, default: str) -> str:
        return str(self._node.declare_parameter(name, default).value)
