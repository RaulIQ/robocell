"""Tests for safe following transitions around target loss and cancellation."""

from hamcrest import assert_that, equal_to
from robocell_sim.pursuit import Pursuit


def test_pursuit_resumes_only_with_a_fresh_observation_after_target_loss() -> None:
    """Loss cancels the active goal and a fresh sighting supplies a new one."""
    lost = Pursuit(True, True, False, True).lose()
    assert_that(
        (lost.cancels(), lost.finish().observe().starts()),
        equal_to((True, True)),
        "following did not replace the cancelled goal after a fresh observation",
    )


def test_pursuit_doesnt_resume_after_manual_cancellation() -> None:
    """An operator cancellation wins over later target observations."""
    assert_that(
        Pursuit(True, True, False, True).toggle().finish().observe().starts(),
        equal_to(False),
        "following resumed despite manual cancellation",
    )


def test_pursuit_doesnt_start_duplicates_while_nav2_accepts_a_goal() -> None:
    """Repeated observations cannot submit duplicate goals during startup."""
    assert_that(
        Pursuit(False, False, False, False).toggle().observe().begin().starts(),
        equal_to(False),
        "following submitted a duplicate goal while Nav2 acceptance was pending",
    )
