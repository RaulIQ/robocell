"""A real virtual CAN bus with two independent MuJoCo drive endpoints."""

import secrets
import threading
from collections.abc import Generator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import final, override

import can
import mujoco
from robocell_sim.hardware.implementations.motor.actuated import Actuated
from robocell_sim.hardware.implementations.motor.attached import Attached
from robocell_sim.hardware.interfaces.drive import Drive

from test.circuit import Circuit


@final
@dataclass(frozen=True)
class Motored(Circuit):
    """Circuit that gives every test its own channel and two physical shafts."""

    _numbers: tuple[int, int]

    @override
    @contextmanager
    def connection(
        self,
    ) -> Generator[
        tuple[can.BusABC, tuple[Drive, Drive], mujoco.MjData, threading.Event]
    ]:
        channel = secrets.token_hex(12)
        pause = threading.Event()
        data = mujoco.MjData(
            mujoco.MjModel.from_xml_string(
                '<mujoco><option timestep="0.002" gravity="0 0 0"/>'
                '<worldbody><body pos="0 0 0"><joint name="left"/>'
                '<geom size="0.1" mass="1" contype="0" conaffinity="0"/></body>'
                '<body pos="1 0 0"><joint name="right"/>'
                '<geom size="0.1" mass="1" contype="0" conaffinity="0"/></body>'
                "</worldbody><actuator>"
                '<velocity name="left" joint="left" kv="1"/>'
                '<velocity name="right" joint="right" kv="1"/></actuator>'
                "</mujoco>"
            )
        )
        with (
            can.Bus(interface="virtual", channel=channel) as host,
            can.Bus(interface="virtual", channel=channel) as left,
            can.Bus(interface="virtual", channel=channel) as right,
            Attached(
                left, self._numbers[0], Actuated(data, "left", "left", pause)
            ).connection() as first,
            Attached(
                right, self._numbers[1], Actuated(data, "right", "right", pause)
            ).connection() as second,
        ):
            yield host, (first, second), data, pause
