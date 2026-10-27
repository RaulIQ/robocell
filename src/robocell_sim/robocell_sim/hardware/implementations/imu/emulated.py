"""IMU board that sends its readings on the CAN bus."""

from dataclasses import dataclass
from typing import final, override

import mujoco

from robocell_sim.hardware.implementations.imu.packed import Packed
from robocell_sim.hardware.interfaces.bus import Bus
from robocell_sim.hardware.interfaces.imu import Imu
from robocell_sim.hardware.interfaces.report import Report

NUMBERS = 256


@final
@dataclass(frozen=True)
class Emulated(Report):
    """Report that plays the IMU board of docs/protocols/imu.md.

    The IMU measures the turn and the acceleration, the data holds the live
    state of the simulation, the period sets in seconds of simulated time
    how often the board measures, and the bus carries the frames. Each
    publication sends one measurement as a gyro frame and then an
    accelerometer frame. Its number counts whole periods of physics steps
    since the start, modulo 256, so it keeps no state and both frames of
    one measurement share it.
    """

    _imu: Imu
    _data: mujoco.MjData
    _period: float
    _bus: Bus

    @override
    def publish(self) -> None:
        for identifier, payload in Packed(
            self._imu.rate(),
            self._imu.acceleration(),
            (
                round(self._data.time / self._data.model.opt.timestep)
                // round(self._period / self._data.model.opt.timestep)
            )
            % NUMBERS,
        ).frames():
            self._bus.send(identifier, payload)
