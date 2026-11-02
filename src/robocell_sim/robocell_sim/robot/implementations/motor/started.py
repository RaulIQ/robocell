"""Fresh host initialization without cached wheel commands."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.motor.assembled import Assembled
from robocell_sim.robot.implementations.motor.observed import Observed
from robocell_sim.robot.implementations.motor.planned import Planned
from robocell_sim.robot.implementations.motor.unwrapped import Unwrapped
from robocell_sim.robot.interfaces.axis import Axis
from robocell_sim.robot.interfaces.pilot import Session
from robocell_sim.robot.interfaces.startup import Startup

_HOST = 127


@final
@dataclass(frozen=True)
class Started(Startup):
    """Startup that reserves node 127 for the host and rejects shared IDs."""

    _numbers: tuple[int, int]
    _axes: tuple[Axis, ...]

    @override
    def session(self, moment: float, *, recovery: bool) -> Session:
        if len(set(self._numbers)) != len(self._numbers) or not all(
            1 <= number < _HOST for number in self._numbers
        ):
            error = "Wheel nodes must be distinct and between 1 and 126"
            raise ValueError(error)
        return (
            tuple(
                self._axes[offset].restart(
                    Planned(number).negotiation(recovery=recovery)
                )
                if self._axes
                else Observed(
                    Planned(number).negotiation(recovery=recovery),
                    Assembled((), moment),
                    Unwrapped((0, 0, -1), _valid=False),
                    (),
                )
                for offset, number in enumerate(self._numbers)
            ),
            ("starting", moment + 5.0, False),
            (-1.0, -1.0, -1.0, -1),
            (0.0, 0.0, -1.0),
        )
