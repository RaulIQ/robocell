"""Ride of the simulated TurtleBot3 Waffle Pi."""

from dataclasses import dataclass
from typing import final, override

import mujoco
import numpy as np

from test.ride import Ride


@final
@dataclass(frozen=True)
class Waffle(Ride):
    """Waffle Pi that starts at rest and drives three seconds on fixed wheels.

    The model holds the scene, the left and right speeds command the wheel
    actuators in rad/s, and the heading turns the robot around the vertical
    axis in radians before it starts.
    """

    _model: mujoco.MjModel
    _left: float
    _right: float
    _heading: float

    @override
    def state(self) -> mujoco.MjData:
        data = mujoco.MjData(self._model)
        mujoco.mju_axisAngle2Quat(
            data.joint("base").qpos[3:],
            np.array([0.0, 0.0, 1.0]),
            self._heading,
        )
        data.ctrl = (self._left, self._right)
        mujoco.mj_step(self._model, data, nstep=1500)
        return data

    @override
    def velocity(self) -> tuple[float, float]:
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(
            self._model,
            self.state(),
            mujoco.mjtObj.mjOBJ_XBODY,
            self._model.body("base_link").id,
            velocity,
            1,
        )
        return (velocity[3], velocity[2])

    @override
    def wheels(self) -> tuple[float, float]:
        data = self.state()
        return (
            data.joint("wheel_left_joint").qvel[0],
            data.joint("wheel_right_joint").qvel[0],
        )
