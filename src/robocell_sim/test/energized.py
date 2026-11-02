"""A pair of drives enabled through their independent CAN endpoints."""

import itertools
import struct
import threading
import time
from collections.abc import Generator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import final, override

import can
import mujoco
from robocell_sim.hardware.interfaces.drive import Drive

from test.circuit import Circuit
from test.motored import Motored


@final
@dataclass(frozen=True)
class Energized(Circuit):
    """Circuit that performs the real wire enable sequence before yielding."""

    _numbers: tuple[int, int]
    _speeds: tuple[int, int]

    @override
    @contextmanager
    def connection(
        self,
    ) -> Generator[
        tuple[can.BusABC, tuple[Drive, Drive], mujoco.MjData, threading.Event]
    ]:
        with Motored(self._numbers).connection() as circuit:
            host, devices, _, _ = circuit
            host.send(
                can.Message(
                    arbitration_id=0, data=b"\x01\0", is_extended_id=False
                )
            )
            host.send(
                can.Message(
                    arbitration_id=0x77F, data=b"\x05", is_extended_id=False
                )
            )
            for word, (number, speed) in itertools.product(
                (6, 7, 15, 15), zip(self._numbers, self._speeds, strict=True)
            ):
                host.send(
                    can.Message(
                        arbitration_id=0x200 + number,
                        data=struct.pack("<Hi", word, speed),
                        is_extended_id=False,
                    )
                )
            for device in devices:
                device.advance(time.monotonic())
            tuple(iter(lambda: host.recv(0), None))
            yield circuit
