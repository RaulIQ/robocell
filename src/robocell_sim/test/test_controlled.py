"""Tests for the immutable drive control state machine."""

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.motor.controlled import Controlled


def test_controlled_dont_apply_the_target_that_enables_the_drive() -> None:
    """Enabling cannot replay a target received before permission exists."""
    assert_that(
        Controlled(0x40, 0, 0, (0, 0.0, 0.0, 0.0))
        .command(6, 0, 1.0, ready=True)
        .command(7, 0, 1.01, ready=True)
        .command(15, 1347, 1.02, ready=True)
        .reading(),
        equal_to((0x27, 0, 0)),
        "drive did not discard the enabling frame's target",
    )


def test_controlled_accept_a_fresh_target_after_enable() -> None:
    """Only an already enabled drive applies signed velocity."""
    assert_that(
        Controlled(0x27, 0, 0, (15, 1.0, 1.0, 0.0))
        .command(15, -2347, 1.02, ready=True)
        .reading(),
        equal_to((0x27, -2347, 0)),
        "enabled drive did not accept a fresh negative target",
    )


def test_controlled_cannot_skip_the_enable_sequence() -> None:
    """An enable word cannot jump over ready and switched-on states."""
    assert_that(
        Controlled(0x40, 0, 0, (0, 0.0, 0.0, 0.0))
        .command(15, 1743, 1.0, ready=True)
        .reading(),
        equal_to((0x40, 0, 0)),
        "disabled drive did not reject an enable shortcut",
    )


def test_controlled_cannot_enable_without_a_live_host() -> None:
    """A healthy communication state is required before enable."""
    assert_that(
        Controlled(0x23, 0, 0, (7, 0.0, 0.0, 0.0))
        .command(15, 0, 1.0, ready=False)
        .reading(),
        equal_to((0x23, 0, 0)),
        "drive did not reject enable without a live host",
    )


@pytest.mark.parametrize(
    ("word", "target"), [(0x10F, 0), (15, 7881), (15, -7881)]
)
def test_controlled_dont_refresh_the_watchdog_with_invalid_commands(
    word: int,
    target: int,
) -> None:
    """Malformed commands cannot prolong a previously accepted target."""
    assert_that(
        Controlled(0x27, 1937, 0, (15, 1.0, 1.19, 0.0))
        .command(word, target, 1.19, ready=True)
        .advance(1.201, 1.937, 0.2, 0.3)
        .reading(),
        equal_to((0x0F, 0, 0xFF01)),
        "invalid command did not leave the watchdog deadline intact",
    )


def test_controlled_stop_when_commands_expire() -> None:
    """A command gap latches a fault even with a healthy heartbeat."""
    assert_that(
        Controlled(0x27, 2743, 0, (15, 1.0, 1.19, 0.0))
        .advance(1.201, 2.743, 0.2, 0.3)
        .reading(),
        equal_to((0x0F, 0, 0xFF01)),
        "drive did not zero its target on watchdog expiry",
    )


def test_controlled_stop_when_heartbeat_expires() -> None:
    """Fresh motor commands cannot conceal a dead host heartbeat."""
    assert_that(
        Controlled(0x27, 1937, 0, (15, 1.29, 1.0, 0.0))
        .advance(1.301, 1.937, 0.2, 0.3)
        .reading(),
        equal_to((0x0F, 0, 0x8130)),
        "drive did not fault on heartbeat loss",
    )


def test_controlled_quick_stop_zeroes_the_target_before_the_wheel_stops() -> (
    None
):
    """Quick-stop inhibits output while physical velocity can remain nonzero."""
    assert_that(
        Controlled(0x27, 3257, 0, (15, 1.0, 1.0, 0.0))
        .command(2, 0, 1.02, ready=True)
        .reading(),
        equal_to((7, 0, 0)),
        "quick-stop did not immediately zero the target",
    )


def test_controlled_disable_after_quick_stop_finishes() -> None:
    """Stopping cannot silently reenable the old velocity."""
    assert_that(
        Controlled(7, 0, 0, (2, 1.0, 1.0, 1.0))
        .advance(1.03, 0.0, 0.2, 0.3)
        .reading(),
        equal_to((0x40, 0, 0)),
        "stopped drive did not require a new enable sequence",
    )


def test_controlled_reset_cannot_clear_an_active_fault() -> None:
    """A reset cannot mask an absent host."""
    assert_that(
        Controlled(8, 0, 0x8130, (0, 1.0, 1.0, 0.0))
        .command(128, 0, 1.5, ready=False)
        .reading(),
        equal_to((8, 0, 0x8130)),
        "reset did not preserve the active heartbeat fault",
    )


def test_controlled_reset_requires_a_new_enable_sequence() -> None:
    """A valid reset clears a recovered fault without restoring motion."""
    assert_that(
        Controlled(8, 0, 0xFF01, (0, 1.0, 1.49, 0.0))
        .command(128, 0, 1.5, ready=True)
        .reading(),
        equal_to((0x40, 0, 0)),
        "fault reset did not leave the drive disabled",
    )
