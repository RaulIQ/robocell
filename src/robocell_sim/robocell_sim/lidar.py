#!/usr/bin/env python3
"""Driver of a standard RPLIDAR A1 connected through a serial port."""

import contextlib
import logging
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException

from robocell_sim.ros.implementations.lidar.session import Session
from robocell_sim.ros.implementations.settings.declared import Declared
from robocell_sim.ros.implementations.wiring.wired import Wired

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("lidar")
rclpy.init()
node = rclpy.create_node("lidar")
settings = Declared(node)
path = settings.text("port", str(Path.home() / ".ros" / "robocell-lidar"))
frame = settings.text("frame_id", "base_scan")
limits = (
    settings.number("range_min", 0.15),
    settings.number("range_max", 12.0),
)
timeout = settings.number("connection_timeout", 10.0)
session = Session(node, path, (Wired(node).scan(), frame, limits))
deadline = time.monotonic() + timeout
with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
    while node.context.ok():
        try:
            attempt = time.monotonic()
            session.publish()
        except (OSError, TimeoutError, ValueError, BufferError) as failure:
            if time.monotonic() - attempt > timeout:
                deadline = time.monotonic() + timeout
            if time.monotonic() >= deadline:
                raise
            log.warning(
                "RPLIDAR session on %s will reconnect because %s", path, failure
            )
            rclpy.spin_once(node, timeout_sec=0)
            time.sleep(0.1)
node.context.try_shutdown()
