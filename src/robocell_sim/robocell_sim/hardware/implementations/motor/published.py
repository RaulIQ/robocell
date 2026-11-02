"""Fault announcements and simulated-time feedback scheduling."""

from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.implementations.motor.reported import Reported
from robocell_sim.hardware.implementations.motor.sampled import Sampled
from robocell_sim.hardware.interfaces.progression import Progression
from robocell_sim.hardware.interfaces.session import Session
from robocell_sim.hardware.interfaces.shaft import Shaft

_PERIOD = 0.02


@final
@dataclass(frozen=True)
class Published(Progression):
    """Publication that never acquires feedback while physics is paused."""

    _node: canopen.LocalNode
    _shaft: Shaft
    _session: Session

    @override
    def advance(self, moment: float) -> Session:
        motor, (host, sample, number), announced = self._session
        Reported(self._node, motor).publish()
        if (
            motor.reading()[2] != announced
            and self._node.nmt.state != "STOPPED"
        ):
            self._node.emcy.send(
                motor.reading()[2],
                int.from_bytes(self._node.get_data(0x1001, 0), "little"),
            )
            announced = motor.reading()[2]
        stamp = self._shaft.measurement()[2]
        if all(
            (
                not self._shaft.paused(),
                self._node.nmt.state == "OPERATIONAL",
                sample < 0 or stamp - sample >= _PERIOD - 1e-12,
            )
        ):
            Sampled(self._node, self._shaft, number).publish()
            sample, number = stamp, (number + 1) % 65536
        return (motor, (host, sample, number), announced)
