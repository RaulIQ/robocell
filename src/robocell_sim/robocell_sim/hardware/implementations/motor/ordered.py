"""Controlword transitions for the synthetic CiA 402 subset."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.decision import Decision, Outcome

_FAULT = 8
_RESET = 128
_LOCAL = 0xFF02
_ENABLED = 0x27
_SWITCHED = 0x23
_ENABLE = 15
_QUICK = 2
_TRANSITIONS = {
    (0x40, 6): 0x21,
    (0x21, 7): 0x23,
    (0x23, 6): 0x21,
    (0x27, 6): 0x21,
    (0x27, 7): 0x23,
    (0x27, 2): 7,
    (0x23, 2): 0x40,
    (0x21, 2): 0x40,
}


@final
@dataclass(frozen=True)
class Ordered(Decision):
    """Decision that cannot skip enable stages or replay an enabling target."""

    _reading: tuple[int, int, int]
    _history: tuple[int, float, float, float]
    _order: tuple[int, int, float]
    _ready: bool

    @override
    def result(self) -> Outcome:
        word, target, moment = self._order
        status, output, fault = self._reading
        history = (
            word,
            moment,
            self._history[2],
            moment
            if word == _QUICK and status == _ENABLED
            else self._history[3],
        )
        if fault:
            reading = (
                (0x40, 0, 0)
                if all(
                    (
                        status == _FAULT,
                        word == _RESET,
                        not self._history[0] & _RESET,
                        self._ready,
                        target == 0,
                        fault != _LOCAL,
                    )
                )
                else self._reading
            )
        else:
            state = (
                0x40
                if word == 0
                else _ENABLED
                if status == _SWITCHED and word == _ENABLE and self._ready
                else _TRANSITIONS.get((status, word), status)
            )
            reading = (
                state,
                target
                if status == state == _ENABLED and word == _ENABLE
                else 0
                if state != status
                else output,
                0,
            )
        return (reading, history)
