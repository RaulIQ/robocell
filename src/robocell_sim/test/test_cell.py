"""Tests for the robocell cell scene."""

import secrets
from pathlib import Path

import mujoco
import numpy as np
import pytest
from hamcrest import assert_that, close_to


@pytest.mark.parametrize("name", ["bin_red", "bin_green", "bin_blue"])
def test_bin_holds_dropped_cube(name: str) -> None:
    """Cube dropped above a bin comes to rest on its floor."""
    spec = mujoco.MjSpec.from_file(
        str(Path(__file__).parents[1] / "models" / "panda" / "cell.xml")
    )
    rng = secrets.SystemRandom()
    spec.body("cube_red").pos = spec.body(name).pos + np.array(
        [
            rng.uniform(-0.02, 0.02),
            rng.uniform(-0.02, 0.02),
            rng.uniform(0.12, 0.25),
        ]
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=1500)
    assert_that(
        data.body("cube_red").xpos[2],
        close_to(0.03, 0.005),
        "cube did not come to rest on the bin floor",
    )
