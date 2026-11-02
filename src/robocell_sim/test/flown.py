"""One physics thread and an independent host connected through CAN."""

import time
from dataclasses import dataclass
from typing import final, override

import mujoco
from robocell_sim.hardware.interfaces.drive import Drive
from robocell_sim.robot.interfaces.pilot import Joints, Pilot

from test.flight import Flight


@final
@dataclass(frozen=True)
class Flown(Flight):
    """Experiment that never advances physics without drive protection."""

    _pilot: Pilot
    _devices: tuple[Drive, ...]
    _data: mujoco.MjData
    _external: bool

    @override
    def enable(self) -> str:
        deadline = time.monotonic() + 4.0
        status = "starting"
        while time.monotonic() < deadline and status == "starting":
            status, _ = self.step(())
        return status

    @override
    def advance(
        self, duration: float, command: tuple[tuple[float, float], ...]
    ) -> tuple[str, tuple[Joints, ...]]:
        deadline = time.monotonic() + duration
        result: tuple[str, tuple[Joints, ...]] = ("starting", ())
        while time.monotonic() < deadline:
            result = self.step(command)
        return result

    @override
    def step(
        self, command: tuple[tuple[float, float], ...]
    ) -> tuple[str, tuple[Joints, ...]]:
        for device in self._devices:
            device.advance(time.monotonic())
        for velocity in command:
            self._pilot.command(velocity, time.monotonic())
        result = self._pilot.advance(
            time.monotonic(),
            round(self._data.time * 1e9) if self._external else -1,
        )
        mujoco.mj_step(self._data.model, self._data)
        time.sleep(0.002)
        return result
