"""Wheel feedback paired by acquisition time, never reception order."""

import struct
from typing import TYPE_CHECKING

import can
import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.motor.matched import Matched
from robocell_sim.robot.implementations.motor.started import Started

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.axis import Axis


@pytest.mark.parametrize(
    ("skew", "moment", "count"),
    [(0, 1.01, 1), (2000000, 1.01, 1), (2000001, 1.01, 0), (0, 1.101, 0)],
)
def test_matched_rejects_different_acquisitions_and_expired_samples(
    skew: int, moment: float, count: int
) -> None:
    """Only complete wheel samples within one physics step can pair."""
    session = Started((17, 29), ()).session(1.0, recovery=False)
    axes: list[Axis] = []
    for offset, stamp in enumerate((123456789, 123456789 + skew)):
        axis = session[0][offset]
        for part, payload in (
            (1, struct.pack("<HiH", 39, -2371, 61937)),
            (2, struct.pack("<iH", -1937, 61937)),
            (3, struct.pack("<Q", stamp)),
        ):
            axis = axis.read(
                can.Message(
                    arbitration_id=0x180 + 17 + offset * 12,
                    data=payload,
                    is_extended_id=False,
                ),
                part,
                1.0,
            )
        axes.append(axis)
    assert_that(
        len(Matched((tuple(axes), *session[1:]), 0.002).joints(moment)),
        equal_to(count),
        "unmatched or expired acquisitions did not prevent wheel pairing",
    )
