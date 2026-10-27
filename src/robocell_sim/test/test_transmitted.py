"""Tests for the bus that transmits frames through python-can."""

import secrets

import can
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.can.transmitted import Transmitted


def test_transmitted_puts_the_frame_on_the_bus() -> None:
    """A listener on the same channel hears the identifier and payload."""
    channel = secrets.token_hex(8)
    identifier = secrets.SystemRandom().randint(0, 0x7FF)
    payload = secrets.token_bytes(secrets.SystemRandom().randint(0, 8))
    with (
        can.Bus(interface="virtual", channel=channel) as sender,
        can.Bus(interface="virtual", channel=channel) as listener,
    ):
        Transmitted(sender).send(identifier, payload)
        heard = listener.recv(1.0)
    assert_that(
        (heard.arbitration_id, bytes(heard.data), heard.is_extended_id)
        if heard
        else (),
        equal_to((identifier, payload, False)),
        "listener did not hear the frame with a standard identifier",
    )
