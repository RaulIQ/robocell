"""Tests for the frame of the IMU board as it came off the CAN bus."""

import secrets
import struct

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.imu.received import Received


def test_received_tells_a_gyro_frame() -> None:
    """A frame with identifier 0x101 carries the gyro."""
    assert_that(
        Received(0x101, secrets.token_bytes(8)).gyro(),
        "frame 0x101 was not told as a gyro frame",
    )


def test_received_tells_an_accelerometer_frame() -> None:
    """A frame with identifier 0x102 does not carry the gyro."""
    assert_that(
        not Received(0x102, secrets.token_bytes(8)).gyro(),
        "frame 0x102 was not told apart from a gyro frame",
    )


def test_received_reads_the_measurement_number_from_byte_6() -> None:
    """The measurement number is the unsigned byte 6 of the payload."""
    number = secrets.SystemRandom().randint(0, 255)
    assert_that(
        Received(
            0x101,
            secrets.token_bytes(6) + bytes((number, 0)),
        ).number(),
        equal_to(number),
        "frame did not read the measurement number from byte 6",
    )


def test_received_reads_three_signed_little_endian_values() -> None:
    """Bytes 0 to 5 hold x, y and z as int16 in little-endian order."""
    values = (
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
    )
    assert_that(
        Received(0x102, struct.pack("<hhhBx", *values, 7)).values(),
        equal_to(values),
        "frame did not read x, y and z as little-endian int16",
    )


def test_received_counts_the_numbers_skipped_since_the_last() -> None:
    """The gap is how many numbers lie strictly between the last and this."""
    last = secrets.SystemRandom().randint(0, 255)
    skipped = secrets.SystemRandom().randint(0, 254)
    assert_that(
        Received(
            0x101,
            bytes(6) + bytes(((last + skipped + 1) % 256, 0)),
        ).gap(last),
        equal_to(skipped),
        "frame did not count the numbers skipped since the last one",
    )
