"""Frames the emulated IMU board airs on a virtual CAN bus."""

import secrets
from dataclasses import dataclass
from typing import final, override

import can
import mujoco
from robocell_sim.hardware.implementations.can.transmitted import Transmitted
from robocell_sim.hardware.implementations.imu.emulated import Emulated
from robocell_sim.hardware.implementations.imu.sensed import Sensed
from robocell_sim.hardware.implementations.physics.paced import Paced
from robocell_sim.hardware.implementations.physics.periodic import Periodic
from robocell_sim.hardware.implementations.physics.physical import Physical
from robocell_sim.hardware.implementations.physics.reported import Reported

from test.traffic import Traffic


@final
@dataclass(frozen=True)
class Aired(Traffic):
    """Frames a listener hears while the board runs as in the simulator.

    The data holds the simulation the board measures, and the moments are
    the simulated times the physics advance to, one after another. The
    board measures every 10 ms and sends on a virtual bus of its own, so
    nothing outside the test hears it and no vcan0 is needed.
    """

    _data: mujoco.MjData
    _moments: tuple[float, ...]

    @override
    def frames(self) -> tuple[tuple[int, bytes], ...]:
        channel = secrets.token_hex(8)
        with (
            can.Bus(interface="virtual", channel=channel) as sender,
            can.Bus(interface="virtual", channel=channel) as listener,
        ):
            for moment in self._moments:
                Paced(
                    Reported(
                        Physical(self._data.model, self._data),
                        Periodic(
                            self._data,
                            0.01,
                            Emulated(
                                Sensed(
                                    self._data, "imu_gyro", "imu_accelerometer"
                                ),
                                self._data,
                                0.01,
                                Transmitted(sender),
                            ),
                        ),
                    ),
                    self._data,
                ).advance(moment)
            return tuple(
                (message.arbitration_id, bytes(message.data))
                for message in iter(lambda: listener.recv(0), None)
            )
