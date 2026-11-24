"""Tests for moving a simulated target independently of the robot."""

from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to
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
