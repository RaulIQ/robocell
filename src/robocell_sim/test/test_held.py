"""Manual intent follows held keys rather than character repetition."""

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.command.held import Held


@pytest.mark.parametrize(
    ("keys", "velocity"),
    [
        (("Up",), (0.073, 0.0)),
        (("Down",), (-0.073, 0.0)),
        (("Left",), (0.0, 0.317)),
        (("Right",), (0.0, -0.317)),
        (("Up", "Left"), (0.073, 0.317)),
        (("Up", "Down", "Left", "Right"), (0.0, 0.0)),
    ],
)
def test_held_moves_at_selected_speeds(
    keys: tuple[str, ...], velocity: tuple[float, float]
) -> None:
    """Held directions cannot depend on repeated press events."""
    command = Held(frozenset(), frozenset(), 0.073, 0.317)
    for key in keys:
        command = command.press(key)
    assert_that(
        command.velocity(),
        equal_to(velocity),
        "held directions did not produce the selected velocity",
    )


def test_held_stops_a_released_direction() -> None:
    """Releasing forward cannot discard a still-held turn."""
    assert_that(
        Held(frozenset(), frozenset(), 0.073, 0.317)
        .press("Up")
        .press("Left")
        .release("Up")
        .velocity(),
        equal_to((0.0, 0.317)),
        "release did not stop only the released direction",
    )


def test_held_keeps_speed_changes_independent_of_repeat() -> None:
    """Repeated key events cannot increase a selected speed repeatedly."""
    assert_that(
        Held(frozenset(), frozenset(), 0.073, 0.317)
        .press("w")
        .press("w")
        .speeds(),
        equal_to((0.083, 0.317)),
        "repeated speed key did not preserve a single adjustment",
    )


def test_held_allows_another_speed_adjustment_after_release() -> None:
    """A second real press cannot be mistaken for key repeat."""
    assert_that(
        Held(frozenset(), frozenset(), 0.073, 0.317)
        .press("e")
        .release("e")
        .press("e")
        .speeds(),
        equal_to((0.073, 0.517)),
        "released speed key did not allow another adjustment",
    )


@pytest.mark.parametrize(
    ("key", "speed", "turn", "expected"),
    [
        ("w", 0.257, 0.317, (0.26, 0.317)),
        ("x", 0.013, 0.317, (0.01, 0.317)),
        ("e", 0.073, 1.773, (0.073, 1.8)),
        ("c", 0.073, 0.137, (0.073, 0.1)),
    ],
)
def test_held_bounds_speed_adjustments(
    key: str, speed: float, turn: float, expected: tuple[float, float]
) -> None:
    """Speed adjustment cannot reverse direction or exceed robot limits."""
    assert_that(
        Held(frozenset(), frozenset(), speed, turn).press(key).speeds(),
        equal_to(expected),
        "speed adjustment did not respect positive bounded settings",
    )


def test_held_cannot_resume_from_a_repeat_after_focus_loss() -> None:
    """A cancelled held key cannot restart motion when focus returns."""
    assert_that(
        Held(frozenset(), frozenset(), 0.073, 0.317)
        .press("Up")
        .press("Escape")
        .press("Up")
        .velocity(),
        equal_to((0.0, 0.0)),
        "focus reset did not inhibit repeated movement keys",
    )


def test_held_requires_release_and_a_new_press_after_focus_loss() -> None:
    """A genuine new press restores control after cancellation."""
    assert_that(
        Held(frozenset(), frozenset(), 0.073, 0.317)
        .press("Up")
        .press("Escape")
        .release("Up")
        .press("Up")
        .velocity(),
        equal_to((0.073, 0.0)),
        "focus reset did not permit a genuinely fresh movement press",
    )
