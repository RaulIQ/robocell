"""A standard A1 session with orderly stop on every exit path."""

import contextlib
import queue
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, final, override

from rclpy.node import Node

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.implementations.lidar.assembled import Assembled
from robocell_sim.robot.implementations.lidar.listening import Listening
from robocell_sim.robot.implementations.lidar.questioned import Questioned
from robocell_sim.ros.implementations.lidar.polling import Polling
from robocell_sim.ros.interfaces.topic import Topic

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.reception import Reception


@final
@dataclass(frozen=True)
class Operating(Report):
    """Report that negotiates one session and stops it before reconnecting."""

    _node: Node
    _port: Port
    _output: tuple[Topic, str, tuple[float, float]]

    @override
    def publish(self) -> None:
        connection = Questioned(self._port, 1.0)
        try:
            period = connection.start()
            state: queue.SimpleQueue[Reception] = queue.SimpleQueue()
            state.put(
                Listening(
                    (b"", True),
                    Assembled((), (0, 0), period, self._output[2]),
                    (period, *self._output[2]),
                    (
                        time.monotonic(),
                        self._node.get_clock().now().nanoseconds,
                        time.monotonic(),
                    ),
                )
            )
            Polling(self._node, self._port, state, self._output).publish()
        finally:
            with contextlib.suppress(OSError, TimeoutError):
                connection.stop()
