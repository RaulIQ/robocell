"""Tests for paced serial delivery and bounded backpressure."""

import secrets

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.uart.queued import Queued

from test.narrow import Narrow


def test_queued_retains_a_partial_write_without_reordering() -> None:
    """A short write leaves its exact tail for the following attempt."""
    payload = secrets.token_bytes(19)
    port = Narrow(bytearray(), 7)
    pending, stamp = Queued(port, payload, 0.0, 64).advance(1.0)
    pending, stamp = Queued(port, pending, stamp, 64).advance(2.0)
    Queued(port, pending, stamp, 64).advance(3.0)
    assert_that(
        port.receive(),
        equal_to(payload),
        "partial writes did not preserve the complete ordered payload",
    )


def test_queued_obeys_the_115200_baud_wire_budget() -> None:
    """Ten milliseconds of 8N1 time cannot carry more than 115 bytes."""
    port = Narrow(bytearray(), 1000)
    Queued(port, secrets.token_bytes(500), 0.0, 1024).advance(0.01)
    assert_that(
        len(port.receive()),
        equal_to(115),
        "delivery did not respect the serial wire budget",
    )


def test_queued_bounds_credit_after_a_long_idle_period() -> None:
    """Idle time cannot permit an entire backlog to burst onto the link."""
    port = Narrow(bytearray(), 1000)
    Queued(port, secrets.token_bytes(500), 0.0, 1024).advance(100.0)
    assert_that(
        len(port.receive()),
        equal_to(256),
        "idle delivery did not bound its accumulated wire credit",
    )


def test_queued_fails_when_the_host_cannot_keep_up() -> None:
    """An unread terminal cannot grow an unbounded pending buffer."""
    with pytest.raises(BufferError) as failure:
        Queued(
            Narrow(bytearray(), 0), secrets.token_bytes(65), 0.0, 64
        ).advance(1.0)
    assert_that(
        str(failure.value),
        equal_to("RPLIDAR UART backlog exceeded its byte limit"),
        "stalled host did not fail with the UART backlog context",
    )
