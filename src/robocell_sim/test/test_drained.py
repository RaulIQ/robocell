"""Tests for the inbox that drains a python-can bus."""

import secrets

import can
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.can.drained import Drained
from robocell_sim.robot.implementations.imu.received import FILTERS


def test_drained_returns_the_frames_in_the_order_they_were_sent() -> None:
    """Every waiting frame comes back once, oldest first."""
    channel = secrets.token_hex(8)
    sent = tuple(
        (
            secrets.SystemRandom().choice((0x101, 0x102)),
            secrets.token_bytes(8),
        )
        for _ in range(secrets.SystemRandom().randint(1, 40))
    )
    with (
        can.Bus(interface="virtual", channel=channel) as sender,
        can.Bus(
            interface="virtual", channel=channel, can_filters=FILTERS
        ) as listener,
    ):
        for identifier, payload in sent:
            sender.send(
                can.Message(
                    arbitration_id=identifier,
                    data=payload,
                    is_extended_id=False,
                )
            )
        heard = Drained(listener).frames()
    assert_that(
        heard,
        equal_to(sent),
        "inbox did not return the frames in the order they were sent",
    )


def test_drained_keeps_out_the_frames_of_other_devices() -> None:
    """The filters of the IMU let no frame of another identifier through."""
    channel = secrets.token_hex(8)
    with (
        can.Bus(interface="virtual", channel=channel) as sender,
        can.Bus(
            interface="virtual", channel=channel, can_filters=FILTERS
        ) as listener,
    ):
        sender.send(
            can.Message(
                arbitration_id=secrets.SystemRandom().choice(
                    (0x080, 0x100, 0x181, 0x200, 0x701)
                ),
                data=secrets.token_bytes(8),
                is_extended_id=False,
            )
        )
        heard = Drained(listener).frames()
    assert_that(
        heard,
        equal_to(()),
        "inbox did not keep out a frame of another device",
    )


def test_drained_returns_nothing_from_an_empty_bus() -> None:
    """An empty bus gives an empty tuple at once, without waiting."""
    with can.Bus(interface="virtual", channel=secrets.token_hex(8)) as bus:
        heard = Drained(bus).frames()
    assert_that(heard, equal_to(()), "empty bus did not give an empty tuple")
