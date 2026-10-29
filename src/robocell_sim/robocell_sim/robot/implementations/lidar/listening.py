"""Reception of standard A1 scan packets."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.lidar.described import Described
from robocell_sim.robot.implementations.lidar.parsed import Parsed
from robocell_sim.robot.implementations.lidar.watched import Watched
from robocell_sim.robot.interfaces.assembly import Assembly
from robocell_sim.robot.interfaces.reception import Reception
from robocell_sim.robot.interfaces.sweep import Sweep


@final
@dataclass(frozen=True)
class Listening(Reception):
    """Reception holding partial bytes, a turn, profile and arrival clock."""

    _pending: tuple[bytes, bool]
    _turn: Assembly
    _profile: tuple[int, float, float]
    _activity: tuple[float, int, float]

    @override
    def read(
        self, payload: bytes, stamp: int, moment: float
    ) -> tuple[Reception, tuple[Sweep, ...]]:
        activity = Watched(self._activity, 2.0).read(payload, stamp, moment)
        pending, waiting = Described(
            self._pending[0] + payload, self._pending[1]
        ).payload()
        events, tail = ((), pending) if waiting else Parsed(pending).split()
        turn = self._turn
        scans: list[Sweep] = []
        for index, echo in enumerate(events):
            turn, ready = turn.accept(
                echo,
                max(
                    0,
                    stamp - (len(events) - index - 1) * self._profile[0] * 1000,
                ),
            )
            scans.extend(ready)
        return (
            Listening((tail, waiting), turn, self._profile, activity),
            tuple(scans),
        )
