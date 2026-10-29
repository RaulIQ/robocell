"""Tests for an A1 device reached through a real pseudo-terminal."""

from hamcrest import assert_that, close_to, equal_to

from test.exchanged import Exchanged


def test_serving_starts_and_stops_the_measurement_stream() -> None:
    """SCAN opens a stream and STOP prevents further acquisition."""
    assert_that(
        tuple(
            len(reply)
            for reply in Exchanged(
                (
                    (0.0, b"", 0.0),
                    (0.0, bytes.fromhex("A5 20"), 0.0),
                    (0.002, b"", 0.0),
                    (0.002, bytes.fromhex("A5 25"), 0.0),
                    (0.004, b"", 0.0),
                ),
                0.1,
            ).responses()
        ),
        equal_to((0, 7, 20, 0, 0)),
        "serial device did not start and stop its measurement stream",
    )


def test_serving_replies_to_a_fragmented_scan_command() -> None:
    """A command split over reads still receives the standard descriptor."""
    assert_that(
        Exchanged(((0.0, b"\xa5", 0.0), (0.0, b"\x20", 0.0)), 0.1).responses(),
        equal_to((b"", bytes.fromhex("A5 5A 05 00 00 40 81"))),
        "serial device did not accept a fragmented SCAN command",
    )


def test_serving_does_not_repeat_measurements_while_paused() -> None:
    """Repeated service at the same simulated time cannot acquire more rays."""
    assert_that(
        tuple(
            len(reply)
            for reply in Exchanged(
                (
                    (0.0, bytes.fromhex("A5 20"), 0.0),
                    (0.002, b"", 0.0),
                    (0.002, b"", 0.0),
                    (0.004, b"", 0.0),
                ),
                0.1,
            ).responses()
        ),
        equal_to((7, 20, 0, 20)),
        "paused device did not preserve its acquisition count",
    )


def test_serving_recovers_from_reset_with_a_new_scan() -> None:
    """RESET leaves the device idle until the next SCAN starts a new session."""
    assert_that(
        tuple(
            len(reply)
            for reply in Exchanged(
                (
                    (0.0, bytes.fromhex("A5 20"), 0.0),
                    (0.002, b"", 0.0),
                    (0.002, bytes.fromhex("A5 40"), 0.0),
                    (0.004, b"", 0.0),
                    (0.004, bytes.fromhex("A5 20"), 0.0),
                    (0.006, b"", 0.0),
                ),
                0.1,
            ).responses()
        ),
        equal_to((7, 20, 0, 0, 7, 20)),
        "reset device did not wait for a fresh SCAN session",
    )


def test_serving_marks_each_rotation_in_the_wire_stream() -> None:
    """An independent reader sees start markers on the first and wrapped ray."""
    payload = Exchanged(
        ((0.0, bytes.fromhex("A5 20"), 0.0), (0.2, b"", 0.0)), 0.6
    ).responses()[1]
    assert_that(
        [
            index // 5
            for index in range(0, len(payload), 5)
            if payload[index] & 1
        ],
        equal_to([0, 381]),
        "serial stream did not mark the first ray of each rotation",
    )


def test_serving_measures_motion_within_a_rotation() -> None:
    """Successive rays see a changed robot position before a turn completes."""
    replies = Exchanged(
        (
            (0.0, bytes.fromhex("A5 20"), 0.0),
            (0.000476, b"", 0.0),
            (0.000952, b"", 0.237),
        ),
        0.1,
    ).responses()
    assert_that(
        (
            int.from_bytes(replies[1][3:5], "little")
            - int.from_bytes(replies[2][3:5], "little")
        )
        / 4000,
        close_to(0.237, 0.002),
        "sequential rays did not see movement within a rotation",
    )


def test_serving_reports_the_period_used_by_its_measurements() -> None:
    """The queried standard period agrees with the number of acquired rays."""
    replies = Exchanged(
        (
            (0.0, bytes.fromhex("A5 59"), 0.0),
            (0.0, bytes.fromhex("A5 20"), 0.0),
            (0.00476, b"", 0.0),
        ),
        0.1,
    ).responses()
    assert_that(
        (int.from_bytes(replies[0][7:9], "little"), len(replies[2]) // 5),
        equal_to((476, 10)),
        "reported sample period did not agree with the measurement stream",
    )


def test_serving_does_not_execute_commands_inside_unsupported_payloads() -> (
    None
):
    """An embedded SCAN in a payload cannot accidentally start the laser."""
    assert_that(
        Exchanged(
            ((0.0, bytes.fromhex("A5 82 02 A5 20 A0"), 0.0), (0.002, b"", 0.0)),
            0.1,
        ).responses(),
        equal_to((b"", b"")),
        "unsupported payload did not remain separate from host commands",
    )


def test_serving_returns_the_standard_health_reply() -> None:
    """The host receives a framed healthy status before scanning."""
    assert_that(
        Exchanged(((0.0, bytes.fromhex("A5 52"), 0.0),), 0.1).responses(),
        equal_to((bytes.fromhex("A5 5A 03 00 00 00 06 00 00 00"),)),
        "serial device did not return the standard healthy reply",
    )


def test_serving_identifies_the_board_as_synthetic() -> None:
    """The device exposes its supported firmware and a synthetic serial."""
    assert_that(
        Exchanged(((0.0, bytes.fromhex("A5 50"), 0.0),), 0.1).responses(),
        equal_to(
            (
                bytes.fromhex("A5 5A 14 00 00 00 04 18 11 01 05")
                + b"ROBOCELL-A1-0001",
            )
        ),
        "serial device did not identify its synthetic standard-SCAN profile",
    )


def test_serving_accepts_force_scan() -> None:
    """A powered motor also produces standard packets after FORCE_SCAN."""
    assert_that(
        tuple(
            len(reply)
            for reply in Exchanged(
                ((0.0, bytes.fromhex("A5 21"), 0.0), (0.001, b"", 0.0)),
                0.1,
            ).responses()
        ),
        equal_to((7, 10)),
        "forced scan did not produce the standard measurement stream",
    )


def test_serving_discards_an_expired_partial_command() -> None:
    """An old prefix cannot turn later garbage into a valid command."""
    assert_that(
        Exchanged(
            (
                (0.0, b"\xa5", 0.0),
                (5.1, b"\x20", 0.0),
                (5.1, bytes.fromhex("A5 20"), 0.0),
            ),
            0.1,
        ).responses(),
        equal_to((b"", b"", bytes.fromhex("A5 5A 05 00 00 40 81"))),
        "expired command prefix did not recover at the next complete request",
    )


def test_serving_restarts_a_scan_without_old_measurements() -> None:
    """A second SCAN sends a new descriptor and resets acquisition counting."""
    assert_that(
        tuple(
            len(reply)
            for reply in Exchanged(
                (
                    (0.0, bytes.fromhex("A5 20"), 0.0),
                    (0.002, b"", 0.0),
                    (0.002, bytes.fromhex("A5 20"), 0.0),
                    (0.002, b"", 0.0),
                    (0.002476, b"", 0.0),
                ),
                0.1,
            ).responses()
        ),
        equal_to((7, 20, 7, 0, 5)),
        "restarted scan did not reset its descriptor and acquisition count",
    )
