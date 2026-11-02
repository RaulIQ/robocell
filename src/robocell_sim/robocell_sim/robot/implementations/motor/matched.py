"""Acquisition-time pairing and feedback supervision."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.motor.supervised import Supervised
from robocell_sim.robot.interfaces.observation import Observation
from robocell_sim.robot.interfaces.pilot import Joints, Session

_PARTIAL = 0.1
_ENABLED = 0x27


@final
@dataclass(frozen=True)
class Matched(Observation):
    """Pair that never publishes wheels from different acquisition epochs."""

    _session: Session
    _tolerance: float

    @override
    def joints(self, moment: float) -> tuple[Joints, ...]:
        samples = tuple(
            axis.readings()[0] for axis in self._session[0] if axis.readings()
        )
        result: tuple[Joints, ...] = ()
        if (
            len(samples) == len(self._session[0])
            and all(moment - sample[2] <= _PARTIAL for sample in samples)
            and abs(samples[0][0][4] - samples[1][0][4])
            <= round(self._tolerance * 1e9)
        ):
            result = (
                (
                    max(sample[0][4] for sample in samples),
                    (samples[0][1], samples[1][1]),
                    (samples[0][0][2] / 1000, samples[1][0][2] / 1000),
                ),
            )
        return result

    @override
    def verify(self, moment: float, clock: int) -> None:
        Supervised(self._session).verify(moment, clock)

    @override
    def disabled(self) -> bool:
        return any(
            axis.readings()
            and axis.readings()[0][2] > self._session[2][2]
            and axis.readings()[0][0][0] & 0x6F != _ENABLED
            for axis in self._session[0]
        )
