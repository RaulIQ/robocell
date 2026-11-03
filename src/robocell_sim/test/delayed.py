"""A physical step with deliberate wall-time cost."""

import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Delayed(Step):
    """Step that represents expensive simulation work without a GUI."""

    _origin: Step
    _duration: float

    @override
    def take(self) -> None:
        time.sleep(self._duration)
        self._origin.take()
