"""Body commands and their source time over ROS."""

import queue
import time
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Twist, TwistStamped  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.node import Node

from robocell_sim.robot.interfaces.control import Control
from robocell_sim.robot.interfaces.pilot import Command
from robocell_sim.ros.implementations.command.stamped import Stamped
from robocell_sim.ros.interfaces.steering import Steering


@final
@dataclass(frozen=True)
class Steered(Steering):
    """One ingress for raw ROS commands and stamped keyboard intent."""

    _node: Node
    _state: queue.SimpleQueue[Control]

    @override
    def listen(self) -> None:
        self._node.create_subscription(Twist, "/cmd_vel", self.command, 1)
        self._node.create_subscription(
            TwistStamped, "/keyboard/cmd_vel", self.keyboard, 1
        )

    @override
    def command(self, message: Twist) -> None:
        self._state.put(
            self._state.get_nowait().steer(
                (message.linear.x, message.angular.z), time.monotonic()
            )
        )

    @override
    def keyboard(self, message: TwistStamped) -> None:
        linear, angular, moment = Stamped(
            message, (time.monotonic(), time.time_ns())
        ).command()
        self._state.put(
            self._state.get_nowait().steer((linear, angular), moment)
        )

    @override
    def reading(self) -> Command:
        control = self._state.get_nowait()
        self._state.put(control)
        return control.reading()
