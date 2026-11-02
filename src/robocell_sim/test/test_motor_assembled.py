"""Coherent CANopen feedback assembled only from complete PDO triples."""

import struct
from typing import TYPE_CHECKING

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.motor.assembled import Assembled

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.feedback import Feedback


def test_motor_assembled_decodes_a_complete_snapshot() -> None:
    """A closing timestamp commits matching status and velocity frames."""
    assert_that(
        Assembled((), 0.0)
        .read(1, struct.pack("<HiH", 39, -2371, 61937), 1.0)
        .read(2, struct.pack("<iH", -1937, 61937), 1.001)
        .read(3, struct.pack("<Q", 123456789), 1.002)
        .measurement(),
        equal_to((39, -2371, -1937, 61937, 123456789)),
        "PDO triple did not preserve its coherent measurement",
    )


@pytest.mark.parametrize(
    "frames",
    [
        ((2, struct.pack("<iH", 1937, 17)), (3, struct.pack("<Q", 713))),
        ((1, struct.pack("<HiH", 39, 2371, 17)), (3, struct.pack("<Q", 713))),
        (
            (1, struct.pack("<HiH", 39, 2371, 17)),
            (2, struct.pack("<iH", 1937, 18)),
            (3, struct.pack("<Q", 713)),
        ),
        (
            (1, struct.pack("<HiH", 39, 2371, 17)),
            (2, struct.pack("<iH", 1937, 17)),
            (1, struct.pack("<HiH", 39, 713, 18)),
            (3, struct.pack("<Q", 719)),
        ),
        ((1, b"\x27"), (2, b"\0" * 6), (3, b"\0" * 8)),
    ],
)
def test_motor_assembled_cannot_commit_incomplete_or_mismatched_frames(
    frames: tuple[tuple[int, bytes], ...],
) -> None:
    """Wrong order, lengths and sample numbers invalidate the assembly."""
    sample: Feedback = Assembled((), 0.0)
    for part, payload in frames:
        sample = sample.read(part, payload, 1.0)
    assert_that(
        sample.complete(),
        equal_to(obj=False),
        "invalid PDOs did not discard the sample",
    )


def test_motor_assembled_expires_partial_feedback() -> None:
    """A late closing frame cannot revive a partial measurement."""
    assert_that(
        Assembled((), 0.0)
        .read(1, struct.pack("<HiH", 39, 2371, 17), 1.0)
        .read(2, struct.pack("<iH", 1937, 17), 1.001)
        .read(3, struct.pack("<Q", 123456789), 1.101)
        .complete(),
        equal_to(obj=False),
        "expired PDO assembly did not discard its timestamp",
    )
