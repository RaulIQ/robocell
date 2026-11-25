"""Tests for the report that publishes the picture of the camera."""

import secrets
from pathlib import Path
from struct import pack
from typing import Any

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.ros.implementations.camera.filmed import Filmed

from test.kept import Kept
from test.painted import Painted


def test_filmed_publishes_the_pixels_the_camera_shot() -> None:
    """The image carries every byte of the shot, row after row."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    camera = Painted(
        (
            secrets.randbelow(256),
            secrets.randbelow(256),
            secrets.randbelow(256),
        ),
        (
            secrets.SystemRandom().randint(16, 640),
            secrets.SystemRandom().randint(12, 480),
        ),
    )
    messages: list[Any] = []
    Filmed(mujoco.MjData(model), camera, Kept(messages), Kept([])).publish()
    assert_that(
        bytes(messages[-1].data),
        equal_to(camera.shot().tobytes()),
        "image did not carry the pixels the camera shot",
    )


def test_filmed_publishes_the_size_of_the_shot() -> None:
    """The image is as tall and as wide as the shot."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    size = (
        secrets.SystemRandom().randint(16, 640),
        secrets.SystemRandom().randint(12, 480),
    )
    messages: list[Any] = []
    Filmed(
        mujoco.MjData(model), Painted((0, 0, 0), size), Kept(messages), Kept([])
    ).publish()
    assert_that(
        (messages[-1].width, messages[-1].height),
        equal_to(size),
        "image did not match the size of the shot",
    )


def test_filmed_packs_three_bytes_per_pixel_in_a_row() -> None:
    """Each row of the image spans three bytes for every pixel in it."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    width = secrets.SystemRandom().randint(16, 640)
    messages: list[Any] = []
    Filmed(
        mujoco.MjData(model),
        Painted((0, 0, 0), (width, 48)),
        Kept(messages),
        Kept([]),
    ).publish()
    assert_that(
        messages[-1].step,
        equal_to(width * 3),
        "image did not pack three bytes per pixel in a row",
    )


def test_filmed_encodes_the_picture_as_rgb() -> None:
    """The image names its bytes red, green and blue, in that order."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    messages: list[Any] = []
    Filmed(
        mujoco.MjData(model),
        Painted((9, 99, 199), (64, 48)),
        Kept(messages),
        Kept([]),
    ).publish()
    assert_that(
        messages[-1].encoding,
        equal_to("rgb8"),
        "image did not encode the picture as rgb8",
    )


def test_filmed_places_the_picture_in_the_optical_frame() -> None:
    """The image lies in the optical frame of the camera in the URDF."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    messages: list[Any] = []
    Filmed(
        mujoco.MjData(model),
        Painted((9, 99, 199), (64, 48)),
        Kept(messages),
        Kept([]),
    ).publish()
    assert_that(
        messages[-1].header.frame_id,
        equal_to("camera_rgb_optical_frame"),
        "image did not lie in the optical frame of the camera",
    )


def test_filmed_stamps_the_picture_with_the_simulated_time() -> None:
    """The image carries the time of the simulation it publishes."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=secrets.SystemRandom().randint(2, 900))
    messages: list[Any] = []
    Filmed(data, Painted((9, 99, 199), (64, 48)), Kept(messages), Kept([])).publish()
    assert_that(
        messages[-1].header.stamp.sec
        + messages[-1].header.stamp.nanosec * 1e-9,
        close_to(data.time, 1e-9),
        "image did not carry the simulated time",
    )


def test_filmed_publishes_depth_aligned_to_the_picture() -> None:
    """The depth image carries a float in meters for every RGB pixel."""
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    data = mujoco.MjData(model)
    data.time = 1.375
    pictures: list[Any] = []
    depths: list[Any] = []
    Filmed(
        data, Painted((17, 92, 153), (11, 7), 0.625), Kept(pictures), Kept(depths)
    ).publish()
    assert_that(
        depths[-1],
        equal_to(
            type(pictures[-1])(
                header=pictures[-1].header,
                width=pictures[-1].width,
                height=pictures[-1].height,
                encoding="32FC1",
                step=44,
                data=pack("<f", 0.625) * 77,
            )
        ),
        "depth did not match the RGB frame or carry float pixels in meters",
    )
