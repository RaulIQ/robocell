"""Protective deadlines before CAN processing and physics stepping."""

from dataclasses import dataclass
from typing import final, override

import canopen

from robocell_sim.hardware.implementations.motor.restarted import Restarted
from robocell_sim.hardware.interfaces.progression import Progression
from robocell_sim.hardware.interfaces.session import Session
from robocell_sim.hardware.interfaces.shaft import Shaft


@final
@dataclass(frozen=True)
class Protected(Progression):
    """Protection that inhibits paused, expired or reset drive sessions."""

    _node: canopen.LocalNode
    _shaft: Shaft
    _session: Session

    @override
    def advance(self, moment: float) -> Session:
        _, velocity, stamp = self._shaft.measurement()
        session = (
            Restarted(self._node, self._session).reset(130, moment)
            if stamp < self._session[1][1]
            else self._session
        )
        motor = session[0]
        if self._node.nmt.state == "OPERATIONAL":
            motor = motor.advance(
                moment,
                velocity,
                int.from_bytes(self._node.get_data(0x2002, 0), "little") / 1000,
                (
                    int.from_bytes(self._node.get_data(0x1016, 1), "little")
                    & 0xFFFF
                )
                / 1000,
            )
        if self._shaft.paused():
            motor = motor.event(-1, moment)
        self._shaft.spin(motor.reading()[1] / 1000)
        return (motor, session[1], session[2])
