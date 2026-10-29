"""A raw 115200-baud serial file descriptor."""

import os
import termios
import tty
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.report import Report


@final
@dataclass(frozen=True)
class Configured(Report):
    """Report that applies the A1 UART line settings to an open descriptor."""

    _descriptor: int

    @override
    def publish(self) -> None:
        os.set_blocking(self._descriptor, False)
        tty.setraw(self._descriptor)
        attributes = termios.tcgetattr(self._descriptor)
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        termios.tcsetattr(self._descriptor, termios.TCSANOW, attributes)
