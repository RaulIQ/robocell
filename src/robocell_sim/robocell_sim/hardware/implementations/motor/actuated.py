"""MuJoCo shaft behind a CANopen drive."""

import math
import threading
from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.interfaces.shaft import Shaft


@final
@dataclass(frozen=True)
class Actuated(Shaft):
    """Shaft that applies targets and acquires its joint's actual motion.

    The owning simulation thread alone accesses MuJoCo, including paused
    service, so CAN callbacks cannot race the physics step.
    """

    _data: mujoco.MjData
    _actuator: str
    _joint: str
    _pause: threading.Event

    @override
    def spin(self, velocity: float) -> None:
        if not math.isfinite(velocity):
            error = f"Wheel {self._joint} received a nonfinite velocity"
            raise ValueError(error)
        self._data.ctrl[self._data.model.actuator(self._actuator).id] = velocity

    @override
    def measurement(self) -> tuple[float, float, float]:
        measurement = (
            float(self._data.joint(self._joint).qpos[0]),
            float(self._data.joint(self._joint).qvel[0]),
            float(self._data.time),
        )
        if (
            not all(math.isfinite(value) for value in measurement)
            or measurement[2] < 0
        ):
            error = f"Wheel {self._joint} has an invalid acquisition"
            raise ValueError(error)
        return measurement

    @override
    def paused(self) -> bool:
        return self._pause.is_set()
