"""A bounded standalone motor simulation loop."""

import time
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from typing import final

from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Driven:
    """Step the wheel emulators and refresh an optional viewer."""

    _step: Step
    _duration: float
    _running: Callable[[], bool]
    _sync: Callable[[], None]

    def __call__(self) -> None:
        """Run until the viewer closes, time expires, or the user interrupts."""
        beginning = time.monotonic()
        with suppress(KeyboardInterrupt):
            while self._running() and (
                self._duration == 0
                or time.monotonic() - beginning < self._duration
            ):
                self._step.take()
                self._sync()
