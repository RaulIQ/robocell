"""Tests for the camera rendered by MuJoCo."""

import math
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.camera.rendered import Rendered


def test_rendered_shoots_at_the_renderer_resolution() -> None:
    """A shot is an RGB image as tall and wide as the renderer."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    height = secrets.SystemRandom().randint(48, 480)
    width = secrets.SystemRandom().randint(64, 640)
    assert_that(
        Rendered(mujoco.Renderer(model, height, width), data, "camera_rgb")
        .shot()
        .shape,
        equal_to((height, width, 3)),
        "shot did not match the renderer resolution",
    )


def test_rendered_follows_the_robot_as_it_turns() -> None:
    """A shot after the robot turns differs from the shot before."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = Rendered(mujoco.Renderer(model, 120, 160), data, "camera_rgb")
    before = camera.shot()
    mujoco.mju_axisAngle2Quat(
        data.joint("base").qpos[3:],
        np.array([0.0, 0.0, 1.0]),
        secrets.SystemRandom().uniform(math.pi / 4, math.pi),
    )
    mujoco.mj_forward(model, data)
    assert_that(
        np.array_equal(camera.shot(), before),
        equal_to(obj=False),
        "shot did not change after the robot turned",
    )


def test_rendered_measures_the_front_of_the_low_pallet_in_meters() -> None:
    """A pixel below the optical axis sees the pallet at its known depth."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "workshop.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, 120, 160) as renderer:
        _, depth = Rendered(renderer, data, "camera_rgb").frame()
    assert_that(
        float(depth[84, 80]),
        close_to(
            float(
                data.geom("pallet").xpos[1]
                - model.geom("pallet").size[1]
                - data.camera("camera_rgb").xpos[1]
            ),
            1e-4,
        ),
        "depth did not measure the front of the low pallet in meters",
    )
