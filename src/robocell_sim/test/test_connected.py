"""Tests for distinguishing a disconnected UART from a quiet one."""

import os
import pty

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.uart.terminal import Terminal
from robocell_sim.robot.implementations.uart.connected import Connected


def test_connected_detects_a_disconnected_peer() -> None:
    """A lost simulator must trigger reconnection even if ROS time is paused."""
    master, slave = pty.openpty()
    with os.fdopen(slave, "rb+", buffering=0):
        os.set_blocking(slave, False)
        os.close(master)
        with pytest.raises(ConnectionError) as failure:
            Connected(Terminal(slave), slave).receive()
    assert_that(
        str(failure.value),
        equal_to("RPLIDAR serial endpoint disconnected"),
        "disconnected UART did not report the lost peer",
    )
