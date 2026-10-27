"""Tests for the inaccuracy that adds a bias and a normal noise to a reading."""

import math
import secrets

import numpy as np
from hamcrest import assert_that, close_to, equal_to, is_not
from robocell_sim.hardware.implementations.imu.gaussian import Gaussian


def test_gaussian_passes_the_value_through_with_no_bias_and_no_noise() -> None:
    """A zero bias and a zero noise leave every axis as it was."""
    value = (
        secrets.SystemRandom().uniform(-30.0, 30.0),
        secrets.SystemRandom().uniform(-30.0, 30.0),
        secrets.SystemRandom().uniform(-30.0, 30.0),
    )
    assert_that(
        Gaussian(
            (0.0, 0.0, 0.0),
            0.0,
            np.random.default_rng(secrets.randbits(64)),
        ).reading(value),
        equal_to(value),
        "inaccuracy did not pass the value through unchanged",
    )


def test_gaussian_adds_the_bias_axis_by_axis_with_no_noise() -> None:
    """With no noise, each axis comes out shifted by its own bias."""
    bias = (
        secrets.SystemRandom().uniform(-0.02, 0.02),
        secrets.SystemRandom().uniform(-0.02, 0.02),
        secrets.SystemRandom().uniform(-0.02, 0.02),
    )
    value = (
        secrets.SystemRandom().uniform(-30.0, 30.0),
        secrets.SystemRandom().uniform(-30.0, 30.0),
        secrets.SystemRandom().uniform(-30.0, 30.0),
    )
    assert_that(
        math.dist(
            Gaussian(
                bias, 0.0, np.random.default_rng(secrets.randbits(64))
            ).reading(value),
            np.add(value, bias),
        ),
        close_to(0.0, 1e-12),
        "inaccuracy did not add the bias to each axis",
    )


def test_gaussian_errs_by_the_bias_on_average() -> None:
    """Over many readings, the mean error on each axis is its bias."""
    bias = (
        secrets.SystemRandom().uniform(-0.02, 0.02),
        secrets.SystemRandom().uniform(-0.02, 0.02),
        secrets.SystemRandom().uniform(-0.02, 0.02),
    )
    noise = secrets.SystemRandom().uniform(0.0005, 0.03)
    inaccuracy = Gaussian(
        bias, noise, np.random.default_rng(secrets.randbits(64))
    )
    assert_that(
        float(
            np.max(
                np.abs(
                    np.mean(
                        [
                            inaccuracy.reading((0.0, 0.0, 0.0))
                            for _ in range(10000)
                        ],
                        axis=0,
                    )
                    - bias
                )
            )
        ),
        close_to(0.0, noise * 0.05),
        "mean error did not match the bias",
    )


def test_gaussian_scatters_by_the_noise() -> None:
    """Over many readings, the error on each axis deviates by the noise."""
    noise = secrets.SystemRandom().uniform(0.0005, 0.03)
    inaccuracy = Gaussian(
        (0.004, -0.007, 0.009),
        noise,
        np.random.default_rng(secrets.randbits(64)),
    )
    assert_that(
        float(
            np.max(
                np.abs(
                    np.std(
                        [
                            inaccuracy.reading((1.3, -2.7, 9.8))
                            for _ in range(10000)
                        ],
                        axis=0,
                    )
                    - noise
                )
            )
        ),
        close_to(0.0, noise * 0.05),
        "error did not scatter by the noise",
    )


def test_gaussian_draws_fresh_noise_for_each_reading() -> None:
    """Two readings of one value differ, since each draws its own noise."""
    inaccuracy = Gaussian(
        (0.006, 0.0, -0.003),
        secrets.SystemRandom().uniform(0.0005, 0.03),
        np.random.default_rng(secrets.randbits(64)),
    )
    value = (0.0, 0.0, secrets.SystemRandom().uniform(-5.0, 5.0))
    assert_that(
        inaccuracy.reading(value),
        is_not(equal_to(inaccuracy.reading(value))),
        "two readings did not carry different noise",
    )
