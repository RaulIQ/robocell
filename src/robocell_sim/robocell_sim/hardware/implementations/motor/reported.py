"""Drive control status in its local object dictionary."""

from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.interfaces.motor import Motor
from robocell_sim.hardware.interfaces.report import Report

_TARGET = 0x60FF
_LOCAL = 0xFF02


@final
@dataclass(frozen=True)
class Reported(Report):
    """Report that exposes protection independently of acquisition."""

    _node: canopen.LocalNode
    _motor: Motor

    @override
    def publish(self) -> None:
        status, target, fault = self._motor.reading()
        for index, size, value in (
            (0x6041, 2, status),
            (0x60FF, 4, target),
            (0x603F, 2, fault),
            (0x1001, 1, 0 if not fault else 1 if fault == _LOCAL else 0x11),
        ):
            self._node.set_data(
                index,
                0,
                value.to_bytes(size, "little", signed=index == _TARGET),
            )
