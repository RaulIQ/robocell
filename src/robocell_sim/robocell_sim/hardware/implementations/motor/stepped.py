"""Paced physical step between protective CAN service cycles."""

import time
from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.drive import Drive
from robocell_sim.hardware.interfaces.shaft import Shaft
from robocell_sim.hardware.interfaces.step import Step


@final
@dataclass(frozen=True)
class Stepped(Step):
    """Step that services both drives before and after advancing physics."""

    _data: mujoco.MjData
    _devices: tuple[Drive, ...]
    _shaft: Shaft

    @override
    def take(self) -> None:
        beginning = time.monotonic()
        for device in self._devices:
            device.advance(beginning)
        if not self._shaft.paused():
            mujoco.mj_step(self._data.model, self._data)
        for device in self._devices:
            device.advance(time.monotonic())
        time.sleep(
            max(
                0.0,
                min(0.01, self._data.model.opt.timestep)
                - (time.monotonic() - beginning),
            )
        )
