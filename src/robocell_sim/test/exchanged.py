"""An A1 exchange through an actual kernel pseudo-terminal."""

import os
import pty
import queue
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, final, override

import mujoco
from robocell_sim.hardware.implementations.lidar.directed import Directed
from robocell_sim.hardware.implementations.lidar.rplidar import Rplidar
from robocell_sim.hardware.implementations.lidar.serving import Serving
from robocell_sim.hardware.implementations.uart.terminal import Terminal

from test.conversation import Conversation
from test.played import Played

if TYPE_CHECKING:
    from robocell_sim.hardware.interfaces.device import Device


@final
@dataclass(frozen=True)
class Exchanged(Conversation):
    """Conversation with independent terminal endpoints and a MuJoCo scene."""

    _events: tuple[tuple[float, bytes, float], ...]
    _timeout: float

    @override
    def responses(self) -> tuple[bytes, ...]:
        data = mujoco.MjData(
            mujoco.MjModel.from_xml_path(
                str(Path(__file__).parents[1] / "models/waffle_pi/walls.xml")
            )
        )
        master, slave = pty.openpty()
        with (
            os.fdopen(master, "rb+", buffering=0),
            os.fdopen(slave, "rb+", buffering=0),
        ):
            tty.setraw(slave)
            os.set_blocking(master, False)
            os.set_blocking(slave, False)
            state: queue.SimpleQueue[tuple[Device, bytes, float]] = (
                queue.SimpleQueue()
            )
            state.put(
                (
                    Rplidar(
                        Directed(data, "base_scan", (0.15, 12.0)),
                        (False, 0.0, 0),
                        (b"", 0.0),
                    ),
                    b"",
                    time.monotonic(),
                )
            )
            return Played(
                data,
                Serving(data, Terminal(master), state, 65536),
                Terminal(slave),
                (self._events, self._timeout),
            ).responses()
