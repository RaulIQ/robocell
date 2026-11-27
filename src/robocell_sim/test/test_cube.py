"""Tests for moving a simulated target independently of the robot."""

from pathlib import Path

import cv2
import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.camera.rendered import Rendered
from robocell_sim.hardware.implementations.physics.cube import Cube


def test_cube_moves_forward_along_its_heading() -> None:
    """Forward input moves the cube along its own heading."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models/waffle_pi/workshop.xml")
    )
    data = mujoco.MjData(model)
    index = model.body("follow_cube").mocapid.item()
    cube = Cube(data, index)
    cube.place()
    start = float(data.mocap_pos[index, 1])
    cube.move((0.19, 0.0), 0.23)
    assert_that(
        float(data.mocap_pos[index, 1]) - start,
        close_to(0.0437, 1e-8),
        "the cube did not move forward along its heading",
    )


def test_cube_carries_the_configured_aruco_marker() -> None:
    """The visible cube provides the marker expected by the detector."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models/waffle_pi/workshop.xml")
    )
    assert_that(
        (model.texture("aruco_0").id >= 0, model.geom("follow_marker").id >= 0),
        equal_to((True, True)),
        "the simulated cube did not carry the configured ArUco marker",
    )


def test_cube_exposes_a_readable_aruco_marker_to_the_rgb_camera() -> None:
    """A marker on the cube is readable in the rendered frame."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models/waffle_pi/workshop.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    data.mocap_pos[model.body("follow_cube").mocapid.item(), 2] = 0.05
    mujoco.mj_forward(model, data)
    image = Rendered(mujoco.Renderer(model, 1080, 1920), data, "camera_rgb")
    _, identifiers, _ = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    ).detectMarkers(image.shot())
    assert_that(
        None if identifiers is None else identifiers.tolist(),
        equal_to([[0]]),
        "the RGB camera did not detect the cube ArUco marker",
    )
