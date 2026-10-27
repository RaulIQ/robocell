"""Tests for the measurement packed into the CAN frames of the IMU board."""

import secrets

from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.packed import Packed


def test_packed_matches_the_example_frames_of_the_protocol() -> None:
    """The example measurement of docs/protocols/imu.md packs to its bytes."""
    assert_that(
        Packed((0.0, 0.0, 0.5), (0.0, 0.0, 9.81), 17).frames(),
        equal_to(
            (
                (0x101, bytes.fromhex("00 00 00 00 F4 01 11 00")),
                (0x102, bytes.fromhex("00 00 00 00 52 26 11 00")),
            )
        ),
        "measurement did not pack to the example frames of the protocol",
    )


def test_packed_sends_a_negative_rate_in_twos_complement() -> None:
    """A rate of -0.3 rad/s goes out as the bytes D4 FE."""
    assert_that(
        Packed((-0.3, 0.0, 0.0), (0.0, 0.0, 0.0), 0).frames()[0][1][0:2],
        equal_to(bytes.fromhex("D4 FE")),
        "negative rate did not go out in two's complement",
    )


def test_packed_clips_a_rate_past_the_top_of_the_range() -> None:
    """A rate past 32.767 rad/s goes out as the largest int16."""
    assert_that(
        Packed(
            (0.0, secrets.SystemRandom().uniform(32.8, 400.0), 0.0),
            (0.0, 0.0, 0.0),
            0,
        ).frames()[0][1][2:4],
        equal_to(bytes.fromhex("FF 7F")),
        "rate past the range was not clipped to the largest int16",
    )


def test_packed_clips_a_rate_past_the_bottom_of_the_range() -> None:
    """A rate below -32.768 rad/s goes out as the smallest int16."""
    assert_that(
        Packed(
            (0.0, secrets.SystemRandom().uniform(-400.0, -32.8), 0.0),
            (0.0, 0.0, 0.0),
            0,
        ).frames()[0][1][2:4],
        equal_to(bytes.fromhex("00 80")),
        "rate past the range was not clipped to the smallest int16",
    )


def test_packed_rounds_to_the_nearest_unit() -> None:
    """An acceleration goes out rounded to the nearest 0.001 m/s²."""
    units = secrets.SystemRandom().randint(-30000, 30000)
    assert_that(
        Packed(
            (0.0, 0.0, 0.0),
            (
                (units + secrets.SystemRandom().uniform(-0.45, 0.45)) / 1000,
                0.0,
                0.0,
            ),
            0,
        ).frames()[1][1][0:2],
        equal_to(units.to_bytes(2, "little", signed=True)),
        "acceleration did not go out rounded to the nearest unit",
    )


def test_packed_gives_both_frames_the_same_number() -> None:
    """The gyro and the accelerometer frame carry one measurement number."""
    number = secrets.SystemRandom().randint(0, 255)
    assert_that(
        [
            payload[6]
            for _, payload in Packed(
                (
                    secrets.SystemRandom().uniform(-5.0, 5.0),
                    secrets.SystemRandom().uniform(-5.0, 5.0),
                    secrets.SystemRandom().uniform(-5.0, 5.0),
                ),
                (
                    secrets.SystemRandom().uniform(-20.0, 20.0),
                    secrets.SystemRandom().uniform(-20.0, 20.0),
                    secrets.SystemRandom().uniform(-20.0, 20.0),
                ),
                number,
            ).frames()
        ],
        equal_to([number, number]),
        "frames of one measurement did not carry the same number",
    )
