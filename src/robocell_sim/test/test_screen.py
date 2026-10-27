"""Tests for the window that shows the robot camera."""

import multiprocessing
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import assert_that, equal_to
from robocell_sim.display.implementations.camera.screen import Screen


def test_screen_queues_the_state_to_film() -> None:
    """Recording sends the joint positions to the window process."""
    queue = multiprocessing.get_context("fork").Queue(maxsize=2)
    state = np.array([secrets.SystemRandom().uniform(-5, 5) for _ in range(9)])
    Screen(
        queue,
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        "camera_rgb",
        (640, 480),
    ).record(state)
    assert_that(
        queue.get(timeout=5).tolist(),
        equal_to(state.tolist()),
        "window did not receive the joint positions to film",
    )


def test_screen_drops_a_state_when_the_window_lags() -> None:
    """A full queue drops the new state instead of holding the simulation."""
    queue = multiprocessing.get_context("fork").Queue(maxsize=1)
    first = np.array([secrets.SystemRandom().uniform(-5, 5) for _ in range(9)])
    screen = Screen(
        queue,
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        "camera_rgb",
        (640, 480),
    )
    screen.record(first)
    screen.record(
        np.array([secrets.SystemRandom().uniform(-5, 5) for _ in range(9)])
    )
    assert_that(
        queue.get(timeout=5).tolist(),
        equal_to(first.tolist()),
        "a full queue did not keep the earlier state",
    )


def test_screen_keeps_the_state_it_was_given() -> None:
    """Changing the state after recording leaves the queued copy alone."""
    queue = multiprocessing.get_context("fork").Queue(maxsize=2)
    state = np.array([secrets.SystemRandom().uniform(-5, 5) for _ in range(9)])
    origin = state.tolist()
    Screen(
        queue,
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        "camera_rgb",
        (640, 480),
    ).record(state)
    state += 1
    assert_that(
        queue.get(timeout=5).tolist(),
        equal_to(origin),
        "queued state did not stay apart from the simulation",
    )
