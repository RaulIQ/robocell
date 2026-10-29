"""Tests for command boundaries in the RPLIDAR byte stream."""

from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.lidar.requested import Requested


def test_requested_preserves_an_incomplete_extended_request() -> None:
    """An incomplete payload cannot leak embedded command bytes."""
    assert_that(
        Requested(bytes.fromhex("A5 82 05 A5 20")).split(),
        equal_to(((), bytes.fromhex("A5 82 05 A5 20"))),
        "incomplete extended request did not retain its original bytes",
    )


def test_requested_recovers_after_garbage_and_unsupported_requests() -> None:
    """The next genuine command survives garbage and an unsupported payload."""
    assert_that(
        Requested(bytes.fromhex("00 FF 21 A5 82 02 A5 20 A0 A5 52 A5")).split(),
        equal_to(((0x52,), b"\xa5")),
        "request stream did not recover at the next genuine command",
    )


def test_requested_handles_a_full_read_without_recursion() -> None:
    """A busy host cannot exhaust the stack by batching many commands."""
    assert_that(
        Requested(bytes.fromhex("A5 52") * 2048).split(),
        equal_to(((0x52,) * 2048, b"")),
        "batched commands did not preserve every complete request",
    )
