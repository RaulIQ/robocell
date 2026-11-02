"""Signed encoder rollover and feedback baseline discontinuities."""

import math

import pytest
from hamcrest import assert_that, close_to
from robocell_sim.robot.implementations.motor.unwrapped import Unwrapped


@pytest.mark.parametrize(
    ("first", "second", "distance"),
    [
        (2147483647, -2147483648, 1),
        (-2147483648, 2147483647, -1),
        (-713, -719, -6),
    ],
)
def test_unwrapped_preserves_the_signed_modular_distance(
    first: int, second: int, distance: int
) -> None:
    """Crossing the signed boundary cannot invent a full counter turn."""
    assert_that(
        Unwrapped((0, 0, -1), _valid=False)
        .read(first, 713)
        .read(second, 20000713)
        .angle(),
        close_to((first + distance) * 2 * math.pi / 4096, 1e-9),
        "counter rollover did not preserve wheel travel",
    )


@pytest.mark.parametrize(
    ("position", "stamp"),
    [(2371, 400000000), (2371, 11), (-2147482935, 20000713)],
)
def test_unwrapped_cannot_integrate_an_unobserved_interval(
    position: int, stamp: int
) -> None:
    """A long gap, backwards clock or half-range jump starts a baseline."""
    assert_that(
        Unwrapped((0, 0, -1), _valid=False)
        .read(713, 719)
        .read(position, stamp)
        .angle(),
        close_to(713 * 2 * math.pi / 4096, 1e-12),
        "invalid interval did not preserve the previous accumulated angle",
    )


def test_unwrapped_retains_observed_travel_after_an_epoch_reset() -> None:
    """A restarted connection retains the accumulated angle as its origin."""
    assert_that(
        Unwrapped((0, 0, -1), _valid=False)
        .read(713, 719)
        .reset()
        .read(-2371, 20000719)
        .read(-2377, 40000719)
        .angle(),
        close_to(707 * 2 * math.pi / 4096, 1e-12),
        "epoch reset did not retain observed travel without an outage jump",
    )
