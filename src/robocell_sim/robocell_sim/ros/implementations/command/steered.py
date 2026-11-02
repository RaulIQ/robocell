"""Independent command freshness and collision observations over ROS."""

import queue
import time
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Twist, TwistStamped  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.interfaces.control import Control
from robocell_sim.robot.interfaces.pilot import Command
from robocell_sim.ros.implementations.command.rays import Rays
from robocell_sim.ros.implementations.command.stamped import Stamped
from robocell_sim.ros.interfaces.steering import Steering


@final
@dataclass(frozen=True)
class Steered(Steering):
    """One ingress for raw ROS commands and stamped keyboard intent."""

    _node: Node
    _state: queue.SimpleQueue[Control]
    _protected: bool

    @override
    def listen(self) -> None:
        self._node.create_subscription(Twist, "/cmd_vel", self.command, 1)
        self._node.create_subscription(
            TwistStamped, "/keyboard/cmd_vel", self.keyboard, 1
        )
        if self._protected:
            self._node.create_subscription(
                LaserScan, "/scan", self.scan, qos_profile_sensor_data
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
    def scan(self, message: LaserScan) -> None:
        ranges = Rays(message).ranges()
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        self._state.put(self._state.get_nowait().scan(ranges, stamp))

    @override
    def reading(self, clock: int) -> Command:
        control = self._state.get_nowait()
        self._state.put(control)
        return control.reading(clock, protected=self._protected)
