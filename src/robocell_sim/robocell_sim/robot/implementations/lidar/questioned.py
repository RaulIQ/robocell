"""A standard A1 connection established from an unknown port state."""

import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.robot.implementations.lidar.asked import Asked
from robocell_sim.robot.implementations.lidar.sampling import Sampling
from robocell_sim.robot.implementations.uart.flushed import Flushed
from robocell_sim.robot.implementations.uart.written import Written
from robocell_sim.robot.interfaces.connection import Connection


@final
@dataclass(frozen=True)
class Questioned(Connection):
    """Connection that drains stale data and checks health before SCAN."""

    _port: Port
    _timeout: float

    @override
    def start(self) -> int:
        self.stop()
        time.sleep(0.01)
        Flushed(self._port, self._timeout).publish()
        if Asked(self._port, self._timeout).answer(
            bytes.fromhex("A5 52"), bytes.fromhex("A5 5A 03 00 00 00 06"), 3
        )[0]:
            message = "RPLIDAR did not report a healthy device"
            raise ValueError(message)
        period = Sampling(
            Asked(self._port, self._timeout).answer(
                bytes.fromhex("A5 59"), bytes.fromhex("A5 5A 04 00 00 00 15"), 4
            )
        ).period()
        Written(self._port, bytes.fromhex("A5 20"), self._timeout).publish()
        return period

    @override
    def stop(self) -> None:
        Written(self._port, bytes.fromhex("A5 25"), self._timeout).publish()
