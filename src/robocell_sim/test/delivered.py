"""Serial bytes delivered through the kernel to a ROS publication."""

import os
import pty
import queue
import select
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, final, override

from robocell_sim.hardware.implementations.uart.terminal import Terminal
from robocell_sim.robot.implementations.lidar.assembled import Assembled
from robocell_sim.robot.implementations.lidar.listening import Listening
from robocell_sim.robot.implementations.uart.configured import Configured
from robocell_sim.ros.implementations.lidar.forwarded import Forwarded
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.instant import Instant
from test.kept import Kept
from test.publications import Publications

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.reception import Reception


@final
@dataclass(frozen=True)
class Delivered(Publications):
    """Publications from fragmented serial chunks and an explicit A1 profile."""

    _chunks: tuple[bytes, ...]
    _limits: tuple[float, float]
    _period: int

    @override
    def messages(self) -> tuple[LaserScan, ...]:
        master, slave = pty.openpty()
        state: queue.SimpleQueue[Reception] = queue.SimpleQueue()
        state.put(
            Listening(
                (b"", True),
                Assembled((), (0, 0), self._period, self._limits),
                (self._period, *self._limits),
                (time.monotonic(), 10_000_000_000, time.monotonic()),
            )
        )
        messages: list[Any] = []
        with (
            os.fdopen(master, "rb+", buffering=0),
            os.fdopen(slave, "rb+", buffering=0),
        ):
            Configured(slave).publish()
            report = Forwarded(
                Terminal(slave),
                state,
                Instant(10.0),
                (Kept(messages), "base_scan", self._limits),
            )
            for chunk in self._chunks:
                os.write(master, chunk)
                select.select([slave], [], [], 1.0)
                report.publish()
        return tuple(messages)
