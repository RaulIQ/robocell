"""Tests for the intrinsics of an ideal pinhole camera."""

import math
import secrets

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.camera.pinhole import Pinhole


def test_pinhole_focuses_by_the_vertical_field_of_view() -> None:
    """The vertical focal length fits half the height into half of fovy."""
    fovy = secrets.SystemRandom().uniform(0.2, 2.5)
    height = secrets.SystemRandom().randint(120, 2160)
    assert_that(
        Pinhole(
            fovy, secrets.SystemRandom().randint(160, 3840), height
        ).matrix()[1][1],
        close_to(height / 2 / math.tan(fovy / 2), 1e-9),
        "vertical focal length did not fit half the height into half of fovy",
    )


def test_pinhole_keeps_the_pixels_square() -> None:
    """Both focal lengths are equal for square pixels."""
    matrix = Pinhole(
        secrets.SystemRandom().uniform(0.2, 2.5),
        secrets.SystemRandom().randint(160, 3840),
        secrets.SystemRandom().randint(120, 2160),
    ).matrix()
    assert_that(
        matrix[0][0],
        equal_to(matrix[1][1]),
        "focal lengths did not stay equal for square pixels",
    )


def test_pinhole_centers_the_principal_point() -> None:
    """The principal point sits in the middle of the image."""
    width = secrets.SystemRandom().randint(160, 3840)
    height = secrets.SystemRandom().randint(120, 2160)
    matrix = Pinhole(secrets.SystemRandom().uniform(0.2, 2.5), width, height)
    assert_that(
        (matrix.matrix()[0][2], matrix.matrix()[1][2]),
        equal_to((width / 2, height / 2)),
        "principal point did not sit in the middle of the image",
    )


def test_pinhole_lays_out_the_matrix_without_skew() -> None:
    """The matrix has no skew and ends with the homogeneous row."""
    matrix = Pinhole(
        secrets.SystemRandom().uniform(0.2, 2.5),
        secrets.SystemRandom().randint(160, 3840),
        secrets.SystemRandom().randint(120, 2160),
    ).matrix()
    assert_that(
        (matrix[0][1], matrix[1][0], matrix[2]),
        equal_to((0.0, 0.0, (0.0, 0.0, 1.0))),
        "matrix did not keep zero skew and the homogeneous row",
    )
