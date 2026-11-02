"""An isolated pair of CAN drives and their physical model."""

import threading
from contextlib import AbstractContextManager
from typing import Protocol

import can
import mujoco
from robocell_sim.hardware.interfaces.drive import Drive


class Circuit(Protocol):
    """Circuit that owns its buses, emulator lifetimes and physics state."""

    def connection(
        self,
    ) -> AbstractContextManager[
        tuple[can.BusABC, tuple[Drive, Drive], mujoco.MjData, threading.Event]
    ]:
        """Connect an independent host and two CAN endpoints."""
        ...
