"""Tests for reconstructing target positions from aligned RGB-D images."""

import numpy as np
import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.projection import MissingDepth, project


def test_projection_uses_the_marker_center_and_median_depth() -> None:
    """The projection reconstructs a metric position in the optical frame."""
    depth = np.full((83, 97), 1730, dtype=np.uint16)
    depth[31:37, 54:61:2] = 0
    assert_that(
        project(
            np.array([[52.0, 28.0], [62.5, 28.0], [62.5, 39.0], [52.0, 39.0]]),
            depth,
            np.array([[93.0, 0.0, 48.0], [0.0, 87.0, 41.0], [0.0, 0.0, 1.0]]),
        ),
        equal_to(((57.25 - 48.0) * 1.73 / 93.0, (33.5 - 41.0) * 1.73 / 87.0, 1.73)),
        "the projection did not reconstruct the calibrated marker position",
    )


def test_projection_rejects_a_marker_without_valid_depth() -> None:
    """A detected outline alone cannot provide a three-dimensional position."""
    with pytest.raises(MissingDepth) as error:
        project(
            np.array([[12.0, 9.0], [22.0, 9.0], [22.0, 19.0], [12.0, 19.0]]),
            np.zeros((31, 37), dtype=np.uint16),
            np.array([[73.0, 0.0, 18.0], [0.0, 71.0, 15.0], [0.0, 0.0, 1.0]]),
        )
    assert_that(
        str(error.value),
        equal_to("Marker requires valid depth"),
        "the projection did not reject missing marker depth",
    )
