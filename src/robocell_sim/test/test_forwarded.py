"""Tests for serial A1 measurements becoming ROS laser scans."""

import math
import struct

from hamcrest import assert_that, equal_to

from test.delivered import Delivered


def test_forwarded_publishes_a_complete_rotation_from_uart() -> None:
    """The UART path discards startup data and publishes the next full turn."""
    assert_that(
        [
            tuple(message.ranges)
            for message in Delivered(
                (
                    bytes.fromhex("A5 5A 05 00 00 40 81"),
                    bytes.fromhex("3D 01 00 A0 0F 3E 01 2D 40 1F"),
                    (
                        bytes.fromhex("3D 01 00 A0 0F 3E 01 2D 40 1F")
                        + bytes.fromhex("3E 01 5A E0 2E 3E 01 87 80 3E")
                    ),
                    bytes.fromhex("3D 01 00 A0 0F"),
                ),
                (0.15, 12.0),
                476,
            ).messages()
        ],
        equal_to([(1.0, 2.0, 3.0, 4.0)]),
        "UART rotation did not become a complete ROS laser scan",
    )


def test_forwarded_accepts_every_byte_in_a_separate_read() -> None:
    """Descriptor and measurement fragments cannot change the resulting scan."""
    turn = bytes.fromhex("3D 01 00 A0 0F 3E 01 2D 40 1F") + bytes.fromhex(
        "3E 01 5A E0 2E 3E 01 87 80 3E"
    )
    payload = bytes.fromhex("A5 5A 05 00 00 40 81") + turn * 2 + turn[:5]
    assert_that(
        [
            tuple(message.ranges)
            for message in Delivered(
                tuple(bytes([value]) for value in payload), (0.15, 12.0), 476
            ).messages()
        ],
        equal_to([(1.0, 2.0, 3.0, 4.0)]),
        "fragmented UART reads did not preserve the completed scan",
    )


def test_forwarded_discards_the_damaged_rotation_before_resynchronizing() -> (
    None
):
    """Invalid check bits invalidate a turn and force fresh boundaries."""
    turn = bytes.fromhex("3D 01 00 A0 0F 3E 01 2D 40 1F") + bytes.fromhex(
        "3E 01 5A E0 2E 3E 01 87 80 3E"
    )
    payload = (
        bytes.fromhex("A5 5A 05 00 00 40 81")
        + turn
        + turn[:5]
        + bytes.fromhex("3C 00 00 00 00")
        + turn[5:]
        + turn * 3
    )
    assert_that(
        [
            tuple(message.ranges)
            for message in Delivered((payload,), (0.15, 12.0), 476).messages()
        ],
        equal_to([(1.0, 2.0, 3.0, 4.0)]),
        "damaged rotation did not get discarded before resynchronization",
    )


def test_forwarded_keeps_clockwise_order_and_positive_acquisition_time() -> (
    None
):
    """ROS angles decrease while acquisition time advances along the array."""
    turn = bytes.fromhex("3D 01 00 A0 0F 3E 01 2D 40 1F") + bytes.fromhex(
        "3E 01 5A E0 2E 3E 01 87 80 3E"
    )
    message = Delivered(
        (bytes.fromhex("A5 5A 05 00 00 40 81") + turn * 2 + turn[:5],),
        (0.15, 12.0),
        476,
    ).messages()[0]
    assert_that(
        (
            message.angle_increment < 0,
            message.time_increment > 0,
            message.header.frame_id,
        ),
        equal_to((True, True, "base_scan")),
        "ROS scan did not preserve clockwise acquisition and its frame",
    )


def test_forwarded_accepts_variable_point_counts() -> None:
    """A scan's angular grid follows received samples rather than 360 slots."""
    count = 13
    turn = b"".join(
        struct.pack(
            "<BHH",
            61 if index == 0 else 62,
            (round(index * 360 / count * 64) << 1) | 1,
            4000 + index * 100,
        )
        for index in range(count)
    )
    assert_that(
        [
            len(message.ranges)
            for message in Delivered(
                (bytes.fromhex("A5 5A 05 00 00 40 81") + turn * 2 + turn[:5],),
                (0.15, 12.0),
                476,
            ).messages()
        ],
        equal_to([count]),
        "scan did not preserve the variable number of received samples",
    )


def test_forwarded_represents_invalid_returns_as_infinity() -> None:
    """A missing echo and an out-of-range distance cannot become obstacles."""
    turn = bytes.fromhex("3D 01 00 A0 0F 02 01 2D 00 00") + bytes.fromhex(
        "3E 01 5A E0 2E 3E 01 87 80 3E"
    )
    assert_that(
        tuple(
            Delivered(
                (bytes.fromhex("A5 5A 05 00 00 40 81") + turn * 2 + turn[:5],),
                (0.15, 2.5),
                476,
            )
            .messages()[0]
            .ranges
        ),
        equal_to((1.0, math.inf, math.inf, math.inf)),
        "invalid returns did not become infinite ranges",
    )
