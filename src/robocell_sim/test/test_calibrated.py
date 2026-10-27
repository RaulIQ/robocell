"""Tests for the report that publishes the calibration of the camera."""

import math
import secrets
from pathlib import Path
from typing import Any

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.camera.pinhole import Pinhole
from robocell_sim.ros.implementations.camera.calibrated import Calibrated

from test.kept import Kept


def test_calibrated_publishes_the_camera_matrix() -> None:
    """The intrinsic matrix k is the camera matrix row after row."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    lens = Pinhole(
        secrets.SystemRandom().uniform(0.3, 2.0),
        secrets.SystemRandom().randint(64, 1280),
        secrets.SystemRandom().randint(48, 960),
    )
    messages: list[Any] = []
    Calibrated(mujoco.MjData(model), lens, (640, 480), Kept(messages)).publish()
    assert_that(
        math.dist(
            messages[-1].k, [value for row in lens.matrix() for value in row]
        ),
        close_to(0.0, 1e-9),
        "camera info did not carry the camera matrix",
    )


def test_calibrated_projects_with_the_camera_matrix_alone() -> None:
    """The projection p is the camera matrix with a zero fourth column."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    lens = Pinhole(
        secrets.SystemRandom().uniform(0.3, 2.0),
        secrets.SystemRandom().randint(64, 1280),
        secrets.SystemRandom().randint(48, 960),
    )
    messages: list[Any] = []
    Calibrated(mujoco.MjData(model), lens, (640, 480), Kept(messages)).publish()
    assert_that(
        math.dist(
            messages[-1].p,
            [value for row in lens.matrix() for value in (*row, 0.0)],
        ),
        close_to(0.0, 1e-9),
        "camera info did not project with the camera matrix alone",
    )


def test_calibrated_leaves_the_image_unrectified() -> None:
    """The rectification r is the identity, as for a single camera."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    messages: list[Any] = []
    Calibrated(
        mujoco.MjData(model),
        Pinhole(secrets.SystemRandom().uniform(0.3, 2.0), 640, 480),
        (640, 480),
        Kept(messages),
    ).publish()
    assert_that(
        list(messages[-1].r),
        equal_to([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
        "camera info did not leave the image unrectified",
    )


def test_calibrated_declares_no_lens_distortion() -> None:
    """The ideal pinhole has a plumb bob model with all terms at zero."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    messages: list[Any] = []
    Calibrated(
        mujoco.MjData(model),
        Pinhole(secrets.SystemRandom().uniform(0.3, 2.0), 640, 480),
        (640, 480),
        Kept(messages),
    ).publish()
    assert_that(
        (messages[-1].distortion_model, list(messages[-1].d)),
        equal_to(("plumb_bob", [0.0] * 5)),
        "camera info did not declare an ideal lens without distortion",
    )


def test_calibrated_publishes_the_size_of_the_image() -> None:
    """The camera info is as wide and as tall as the image it calibrates."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    size = (
        secrets.SystemRandom().randint(64, 1280),
        secrets.SystemRandom().randint(48, 960),
    )
    messages: list[Any] = []
    Calibrated(
        mujoco.MjData(model), Pinhole(0.85, *size), size, Kept(messages)
    ).publish()
    assert_that(
        (messages[-1].width, messages[-1].height),
        equal_to(size),
        "camera info did not match the size of the image",
    )


def test_calibrated_places_the_calibration_in_the_optical_frame() -> None:
    """The camera info lies in the optical frame of the camera in the URDF."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    messages: list[Any] = []
    Calibrated(
        mujoco.MjData(model),
        Pinhole(0.85, 640, 480),
        (640, 480),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].header.frame_id,
        equal_to("camera_rgb_optical_frame"),
        "camera info did not lie in the optical frame of the camera",
    )


def test_calibrated_stamps_the_calibration_with_the_simulated_time() -> None:
    """The camera info carries the time of the simulation it publishes."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=secrets.SystemRandom().randint(2, 900))
    messages: list[Any] = []
    Calibrated(
        data, Pinhole(0.85, 640, 480), (640, 480), Kept(messages)
    ).publish()
    assert_that(
        messages[-1].header.stamp.sec
        + messages[-1].header.stamp.nanosec * 1e-9,
        close_to(data.time, 1e-9),
        "camera info did not carry the simulated time",
    )
