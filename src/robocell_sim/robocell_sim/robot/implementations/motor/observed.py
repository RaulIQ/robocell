"""Independent startup and measurement of one wheel."""

from dataclasses import dataclass, replace
from typing import final, override

import can

from robocell_sim.robot.implementations.motor.assembled import Assembled
from robocell_sim.robot.implementations.motor.recorded import Recorded
from robocell_sim.robot.interfaces.axis import Axis, Reading
from robocell_sim.robot.interfaces.feedback import Feedback
from robocell_sim.robot.interfaces.negotiation import Negotiation
from robocell_sim.robot.interfaces.position import Position
from robocell_sim.robot.interfaces.recording import Acquisition


@final
@dataclass(frozen=True)
class Observed(Axis):
    """Wheel that keeps incomplete PDOs separate from committed travel."""

    _negotiation: Negotiation
    _assembly: Feedback
    _position: Position
    _readings: tuple[Acquisition, ...]

    @override
    def advance(
        self, bus: can.BusABC, moment: float, *, synchronized: bool
    ) -> Axis:
        return replace(
            self,
            _negotiation=self._negotiation.advance(
                bus, moment, synchronized=synchronized
            ),
        )

    @override
    def read(self, message: can.Message, part: int, moment: float) -> Axis:
        result = replace(
            self, _negotiation=self._negotiation.read(message, moment)
        )
        if part:
            assembly = self._assembly.read(part, bytes(message.data), moment)
            result = replace(result, _assembly=assembly)
            if assembly.complete():
                position, readings = Recorded(
                    assembly, self._position, self._readings
                ).commit(moment)
                result = replace(
                    result,
                    _assembly=Assembled((), moment),
                    _position=position,
                    _readings=readings,
                )
        return result

    @override
    def ready(self, moment: float) -> tuple[bool, bool, bool]:
        return (
            self._negotiation.enabled(),
            self._negotiation.healthy(moment),
            self._negotiation.waiting(),
        )

    @override
    def restart(self, negotiation: Negotiation) -> Axis:
        return Observed(
            negotiation, Assembled((), 0.0), self._position.reset(), ()
        )

    @override
    def readings(self) -> tuple[Reading, ...]:
        return tuple(
            (sample, self._position.angle(), moment)
            for sample, moment in self._readings
        )
