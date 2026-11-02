"""Protective service of the independently connected wheel endpoints."""

import time
from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.drive import Drive
from robocell_sim.hardware.interfaces.report import Report


@final
@dataclass(frozen=True)
class Polled(Report):
    """Service owned by the physics thread even when the world is paused."""

    _devices: tuple[Drive, ...]

    @override
    def publish(self) -> None:
        for device in self._devices:
            device.advance(time.monotonic())
