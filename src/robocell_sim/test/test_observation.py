"""Tests for the common target observation contract."""

from hamcrest import assert_that, equal_to
from robocell_sim.observation import Observation


def test_observation_carries_position_frame_timestamp_and_validity() -> None:
    """The observation preserves every input needed by target following."""
    observation = Observation(
        (0.73, -1.28, 2.41), "camera_rgb_optical_frame", 172.25, True
    )
    assert_that(
        (
            observation.position(),
            observation.frame(),
            observation.stamp(),
            observation.valid(),
        ),
        equal_to(((0.73, -1.28, 2.41), "camera_rgb_optical_frame", 172.25, True)),
        "the observation did not preserve the common target contract",
    )
