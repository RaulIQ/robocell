"""Lifetime of a CAN host and its physical wheel endpoints."""

import threading
from contextlib import AbstractContextManager
from typing import Protocol

import can
import mujoco
from robocell_sim.robot.interfaces.pilot import Pilot

from test.flight import Flight


class Journey(Protocol):
    """An isolated physical experiment without ROS or network access."""

    def connection(
        self,
    ) -> AbstractContextManager[
        tuple[Flight, Pilot, mujoco.MjData, can.BusABC, threading.Event]
    ]:
        """Connect a fresh host to two independently addressed drives."""
        ...
