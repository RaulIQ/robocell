"""Separate command, heartbeat and feedback deadlines."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.interfaces.pilot import Session
from robocell_sim.robot.interfaces.supervision import Supervision

_COMMAND = 0.25
_GAP = 300000000
_FAULTS = {8, 15}


@final
@dataclass(frozen=True)
class Supervised(Supervision):
    """Supervision that continues on wall time while physics is paused."""

    _session: Session

    @override
    def verify(self, moment: float, clock: int) -> None:
        if (
            self._session[3][2] > self._session[2][2]
            and moment - self._session[3][2] > _COMMAND
        ):
            error = (
                "Body command expired after 250 ms "
                "and requires a new enable sequence"
            )
            raise TimeoutError(error)
        if clock < self._session[2][3]:
            error = "Simulation clock moved backwards and invalidated feedback"
            raise OSError(error)
        self.feedback(moment, clock)

    @override
    def feedback(self, moment: float, clock: int) -> None:
        if not all(axis.ready(moment)[1] for axis in self._session[0]):
            error = (
                "A drive lost its operational heartbeat "
                "and both wheels must stop"
            )
            raise OSError(error)
        for axis in self._session[0]:
            if clock - (
                axis.readings()[0][0][4]
                if axis.readings()
                else self._session[1][1]
            ) > _GAP or (
                not self._session[1][2]
                and moment
                - (
                    axis.readings()[0][2]
                    if axis.readings()
                    else self._session[2][2]
                )
                > _GAP / 1e9
            ):
                error = (
                    "Drive feedback exceeded its 300 ms supervision deadline"
                )
                raise TimeoutError(error)
            if axis.readings() and axis.readings()[0][0][0] & 0x4F in _FAULTS:
                error = (
                    "Drive reports a latched fault requiring explicit recovery"
                )
                raise OSError(error)
