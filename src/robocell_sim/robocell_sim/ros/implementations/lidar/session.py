"""An open Lidar session connected to its ROS publishers."""

import os
from dataclasses import dataclass
from typing import final, override

from rclpy.node import Node

from robocell_sim.hardware.implementations.uart.terminal import Terminal
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.implementations.uart.configured import Configured
from robocell_sim.robot.implementations.uart.connected import Connected
from robocell_sim.ros.implementations.lidar.operating import Operating
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Session(Report):
    """Open the serial port, configure the lidar and serve ROS scans."""

    _node: Node
    _path: str
    _output: tuple[Topic, str, tuple[float, float]]

    @override
    def publish(self) -> None:
        flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        with os.fdopen(
            os.open(self._path, flags), "rb+", buffering=0
        ) as device:
            descriptor = device.fileno()
            Configured(descriptor).publish()
            Operating(
                self._node,
                Connected(Terminal(descriptor), descriptor),
                self._output,
            ).publish()
