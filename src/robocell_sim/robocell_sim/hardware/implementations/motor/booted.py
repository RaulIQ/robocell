"""Initial CANopen boot of an inhibited physical drive."""

from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.hardware.interfaces.shaft import Shaft


@final
@dataclass(frozen=True)
class Booted(Report):
    """Boot that prevents motion before announcing the node."""

    _node: canopen.LocalNode
    _shaft: Shaft

    @override
    def publish(self) -> None:
        self._shaft.spin(0.0)
        self._node.nmt.send_command(129)
        self._node.nmt.send_command(128)
