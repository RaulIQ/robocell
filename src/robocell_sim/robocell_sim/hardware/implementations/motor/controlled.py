"""Bounded synthetic CiA 402 velocity control."""

from dataclasses import dataclass, replace
from typing import final, override

from robocell_sim.hardware.implementations.motor.expired import Expired
from robocell_sim.hardware.implementations.motor.ordered import Ordered
from robocell_sim.hardware.interfaces.motor import Motor

_WORDS = {0, 2, 6, 7, 15, 128}
_LIMIT = 7880
_REACTION = 0x0F
_QUICK = 7
_STOPPED = 0.01


@final
@dataclass(frozen=True)
class Controlled(Motor):
    """Drive whose immutable state separates output permission from commands.

    History holds the controlword, command and heartbeat reception times,
    and the beginning of quick-stop.
    """

    _status: int
    _target: int
    _fault: int
    _history: tuple[int, float, float, float]

    @override
    def command(
        self, word: int, target: int, moment: float, *, ready: bool
    ) -> Motor:
        if word not in _WORDS or not -_LIMIT <= target <= _LIMIT:
            return self
        reading, history = Ordered(
            self.reading(), self._history, (word, target, moment), _ready=ready
        ).result()
        return Controlled(*reading, history)

    @override
    def event(self, code: int, moment: float) -> Motor:
        result = self
        if code == 0:
            result = replace(
                self,
                _history=(
                    self._history[0],
                    self._history[1],
                    moment,
                    self._history[3],
                ),
            )
        elif code == -1:
            result = replace(
                self, _status=self._status if self._fault else 0x40, _target=0
            )
        elif not self._fault:
            result = replace(self, _status=_REACTION, _target=0, _fault=code)
        return result

    @override
    def advance(
        self, moment: float, velocity: float, watchdog: float, timeout: float
    ) -> Motor:
        status = self._status
        code = (
            0
            if self._fault
            else Expired(
                status, self._history, moment, (watchdog, timeout, velocity)
            ).code()
        )
        if status == _REACTION:
            status = 8
        elif code:
            return self.event(code, moment)
        elif status == _QUICK and abs(velocity) < _STOPPED:
            status = 0x40
        return replace(
            self,
            _status=status,
            _target=0 if status != self._status else self._target,
        )

    @override
    def reading(self) -> tuple[int, int, int]:
        return (self._status, self._target, self._fault)
