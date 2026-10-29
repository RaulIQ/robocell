"""Continuous polling of a standard A1 stream."""

import queue
from dataclasses import dataclass
from typing import final, override

import rclpy
from rclpy.node import Node

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.reception import Reception
from robocell_sim.ros.implementations.clock.timed import Timed
from robocell_sim.ros.implementations.lidar.forwarded import Forwarded
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Polling(Report):
    """Report that receives serial data while servicing ROS callbacks."""

    _node: Node
    _port: Port
    _state: queue.SimpleQueue[Reception]
    _output: tuple[Topic, str, tuple[float, float]]

    @override
    def publish(self) -> None:
        driver = Forwarded(
            self._port, self._state, Timed(self._node), self._output
        )
        while self._node.context.ok():
            rclpy.spin_once(self._node, timeout_sec=0.005)
            driver.publish()
