"""Tests for chronological A1 reception across UART reads."""

from typing import TYPE_CHECKING

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.lidar.assembled import Assembled
from robocell_sim.robot.implementations.lidar.chronological import Chronological
from robocell_sim.robot.implementations.lidar.listening import Listening

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.reception import Reception
    from robocell_sim.robot.interfaces.sweep import Sweep


def test_listening_discards_queued_turns_that_cannot_fit_a_paused_clock() -> (
    None
):
    """Later UART reads cannot republish earlier acquisition intervals."""
    turn = bytes.fromhex(
        "3D 01 00 A0 0F 3E 01 2D 40 1F 3E 01 5A E0 2E 3E 01 87 80 3E"
    )
    reception, first = Listening(
        (b"", True),
        Assembled((), (0, 0), (476, (0.15, 12.0)), Chronological(-1, 0)),
        (476, 0.15, 12.0),
        (0.0, 0, 0.0),
    ).read(
        bytes.fromhex("A5 5A 05 00 00 40 81") + turn * 2 + turn[:5],
        10_000_000_000,
        0.01,
    )
    _, second = reception.read(turn[5:] + turn + turn[:5], 10_000_000_000, 0.02)
    assert_that(
        [scan.timing()[0] for scan in (*first, *second)],
        equal_to([9_998_096_000]),
        "queued turns did not get discarded before their timestamps regressed",
    )


@pytest.mark.parametrize("size", [1, 2, 3, 5, 7, 19, 29, 72])
def test_listening_keeps_acquisition_times_across_uart_fragments(
    size: int,
) -> None:
    """Packet grouping cannot change a continuous sample timeline."""
    turn = bytes.fromhex(
        "3D 01 00 A0 0F 3E 01 2D 40 1F 3E 01 5A E0 2E 3E 01 87 80 3E"
    )
    payload = bytes.fromhex("A5 5A 05 00 00 40 81") + turn * 3 + turn[:5]
    reception: Reception = Listening(
        (b"", True),
        Assembled((), (0, 0), (476, (0.15, 12.0)), Chronological(-1, 0)),
        (476, 0.15, 12.0),
        (0.0, 0, 0.0),
    )
    scans: list[Sweep] = []
    for offset in range(0, len(payload), size):
        end = min(len(payload), offset + size)
        reception, ready = reception.read(
            payload[offset:end],
            10_000_000_000 + max(0, (end - 7) // 5) * 476_000,
            end / 1000,
        )
        scans.extend(ready)
    assert_that(
        [(scan.timing()[0], scan.ranges()) for scan in scans],
        equal_to(
            [
                (10_002_380_000, (1.0, 2.0, 3.0, 4.0)),
                (10_004_284_000, (1.0, 2.0, 3.0, 4.0)),
            ]
        ),
        "UART fragmentation did not preserve acquisition times and ray order",
    )


def test_listening_recovers_after_draining_bytes_during_pause() -> None:
    """Resume cannot label old data with the current arrival time."""
    turn = bytes.fromhex(
        "3D 01 00 A0 0F 3E 01 2D 40 1F 3E 01 5A E0 2E 3E 01 87 80 3E"
    )
    reception, _ = Listening(
        (b"", True),
        Assembled((), (0, 0), (476, (0.15, 12.0)), Chronological(-1, 0)),
        (476, 0.15, 12.0),
        (0.0, 0, 0.0),
    ).read(
        bytes.fromhex("A5 5A 05 00 00 40 81") + turn * 2 + turn[:5],
        10_000_000_000,
        0.01,
    )
    reception, _ = reception.read(
        turn[5:] + turn + turn[:5], 10_000_000_000, 0.02
    )
    _, scans = reception.read(turn[5:] + turn[:5], 10_005_000_000, 0.03)
    assert_that(
        [scan.timing() for scan in scans],
        equal_to([(10_000_000_000, 0.001904, 0.000476)]),
        "resumed acquisition did not preserve the original sample interval",
    )
