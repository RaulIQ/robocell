"""Heartbeat discovery uses the startup budget rather than SDO retries."""

import secrets

import can
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.motor.timed import Timed


def test_motor_timed_waits_for_late_heartbeat_within_startup_budget() -> None:
    """The simulator may start after the host without exhausting SDO retries."""
    steps = ((0x711, b"", b"\x7f"),)
    with can.Bus(interface="virtual", channel=secrets.token_hex(12)) as bus:
        result = Timed(bus, 17, steps, (13.71, 12.71, 3)).renew(13.97)
    assert_that(
        result[0],
        equal_to(steps),
        "heartbeat discovery did not retain the overall startup budget",
    )
