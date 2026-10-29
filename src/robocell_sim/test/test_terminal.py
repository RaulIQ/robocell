"""Tests for nonblocking kernel serial endpoints."""

import os
import pty
import secrets
import select
import tty

from hamcrest import assert_that, equal_to, less_than
from robocell_sim.hardware.implementations.uart.terminal import Terminal


def test_terminal_reads_a_real_peer_without_translation() -> None:
    """A raw pseudo-terminal preserves all byte values including newlines."""
    master, slave = pty.openpty()
    payload = bytes(range(256))
    with (
        os.fdopen(master, "rb+", buffering=0),
        os.fdopen(slave, "rb+", buffering=0),
    ):
        tty.setraw(slave)
        os.set_blocking(master, False)
        os.write(slave, payload)
        select.select([master], [], [], 1.0)
        received = Terminal(master).receive()
    assert_that(
        received,
        equal_to(payload),
        "raw terminal did not preserve the peer's bytes",
    )


def test_terminal_returns_immediately_when_no_input_is_available() -> None:
    """An empty nonblocking terminal cannot suspend the physics thread."""
    master, slave = pty.openpty()
    with (
        os.fdopen(master, "rb+", buffering=0),
        os.fdopen(slave, "rb+", buffering=0),
    ):
        tty.setraw(slave)
        os.set_blocking(master, False)
        received = Terminal(master).receive()
    assert_that(
        received,
        equal_to(b""),
        "empty terminal did not return without input",
    )


def test_terminal_reports_partial_writes_to_a_full_kernel_buffer() -> None:
    """A real terminal reports capacity instead of dropping the unsent tail."""
    master, slave = pty.openpty()
    payload = secrets.token_bytes(65536)
    with (
        os.fdopen(master, "rb+", buffering=0),
        os.fdopen(slave, "rb+", buffering=0),
    ):
        tty.setraw(slave)
        os.set_blocking(master, False)
        count = Terminal(master).send(payload)
    assert_that(
        count,
        less_than(len(payload)),
        "terminal did not report the partial kernel write",
    )
