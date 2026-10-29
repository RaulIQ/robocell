"""UART acquisitions forwarded to a ROS scan topic."""

import queue
import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.clock import Clock
from robocell_sim.robot.interfaces.reception import Reception
from robocell_sim.ros.implementations.lidar.published import Published
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Forwarded(Report):
    """Report connecting a byte port, scan state, clock and ROS output."""

    _port: Port
    _state: queue.SimpleQueue[Reception]
    _clock: Clock
    _output: tuple[Topic, str, tuple[float, float]]

    @override
    def publish(self) -> None:
        reception, scans = self._state.get_nowait().read(
            self._port.receive(), self._clock.now(), time.monotonic()
        )
        self._state.put(reception)
        for scan in scans:
            Published(
                scan, self._output[1], self._output[2], self._output[0]
            ).publish()
