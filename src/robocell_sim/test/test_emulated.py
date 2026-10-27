"""Tests for the emulated IMU board that sends its readings on CAN."""

import itertools
import math
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.imu.gaussian import Gaussian
from robocell_sim.hardware.implementations.imu.noisy import Noisy
from robocell_sim.hardware.implementations.imu.packed import ACCELEROMETER, GYRO
from robocell_sim.hardware.implementations.imu.sensed import Sensed

from test.aired import Aired
from test.waffle import Waffle


def test_emulated_sends_the_yaw_rate_of_a_spinning_robot() -> None:
    """The gyro z of the last frame is the turn rate the wheels demand."""
    speed = secrets.SystemRandom().uniform(2.0, 6.0)
    data = Waffle(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        -speed,
        speed,
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    assert_that(
        int.from_bytes(
            [
                payload
                for identifier, payload in Aired(
                    data,
                    Sensed(data, "imu_gyro", "imu_accelerometer"),
                    (data.time + secrets.SystemRandom().uniform(0.02, 0.5),),
                ).frames()
                if identifier == GYRO
            ][-1][4:6],
            "little",
            signed=True,
        )
        / 1000,
        close_to(2 * speed * 0.033 / 0.288, 2 * speed * 0.033 / 0.288 * 0.1),
        "board did not send the yaw rate of the spinning robot",
    )


def test_emulated_sends_gravity_for_a_robot_at_rest() -> None:
    """The accelerometer z of a robot at rest is about 9810 thousandths."""
    data = Waffle(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        0.0,
        0.0,
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    assert_that(
        int.from_bytes(
            [
                payload
                for identifier, payload in Aired(
                    data,
                    Sensed(data, "imu_gyro", "imu_accelerometer"),
                    (data.time + secrets.SystemRandom().uniform(0.02, 0.5),),
                ).frames()
                if identifier == ACCELEROMETER
            ][-1][4:6],
            "little",
            signed=True,
        ),
        close_to(9810, 50),
        "board did not send gravity for the robot at rest",
    )


def test_emulated_sends_a_gyro_and_an_accelerometer_frame_each_period() -> None:
    """One second of simulated time sends 100 pairs, gyro first in each."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        )
    )
    assert_that(
        [
            identifier
            for identifier, _ in Aired(
                data,
                Sensed(data, "imu_gyro", "imu_accelerometer"),
                (1.0,),
            ).frames()
        ],
        equal_to([GYRO, ACCELEROMETER] * 100),
        "board did not send one gyro and one accelerometer frame per period",
    )


def test_emulated_numbers_each_measurement_one_past_the_last() -> None:
    """The measurement number grows by one every period of simulated time."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        )
    )
    assert_that(
        {
            (later[6] - earlier[6]) % 256
            for earlier, later in itertools.pairwise(
                payload
                for identifier, payload in Aired(
                    data,
                    Sensed(data, "imu_gyro", "imu_accelerometer"),
                    (secrets.SystemRandom().uniform(0.2, 2.0),),
                ).frames()
                if identifier == GYRO
            )
        },
        equal_to({1}),
        "measurement number did not grow by one each period",
    )


def test_emulated_wraps_the_number_from_255_to_0() -> None:
    """The only measurement whose number drops is a 0 after a 255."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        )
    )
    assert_that(
        {
            (earlier[6], later[6])
            for earlier, later in itertools.pairwise(
                payload
                for identifier, payload in Aired(
                    data,
                    Sensed(data, "imu_gyro", "imu_accelerometer"),
                    (secrets.SystemRandom().uniform(2.6, 6.0),),
                ).frames()
                if identifier == GYRO
            )
            if later[6] < earlier[6]
        },
        equal_to({(255, 0)}),
        "measurement number did not wrap from 255 to 0",
    )


def test_emulated_sends_the_same_frames_however_the_clock_is_sliced() -> None:
    """One advance by a second sends what a hundred advances by 10 ms send."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    speed = secrets.SystemRandom().uniform(-6.0, 6.0)
    whole = mujoco.MjData(model)
    whole.ctrl = (-speed, speed)
    sliced = mujoco.MjData(model)
    sliced.ctrl = (-speed, speed)
    assert_that(
        Aired(
            sliced,
            Sensed(sliced, "imu_gyro", "imu_accelerometer"),
            tuple(count * 0.01 for count in range(1, 101)),
        ).frames(),
        equal_to(
            Aired(
                whole, Sensed(whole, "imu_gyro", "imu_accelerometer"), (1.0,)
            ).frames()
        ),
        "board did not send the same frames when the clock advanced in slices",
    )


def test_emulated_sends_the_clean_frames_with_no_bias_and_no_noise() -> None:
    """A board whose sensors read exactly sends what the clean board sends."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    speed = secrets.SystemRandom().uniform(-6.0, 6.0)
    moment = secrets.SystemRandom().uniform(0.05, 1.5)
    clean = mujoco.MjData(model)
    clean.ctrl = (-speed, speed)
    exact = mujoco.MjData(model)
    exact.ctrl = (-speed, speed)
    assert_that(
        Aired(
            exact,
            Noisy(
                Sensed(exact, "imu_gyro", "imu_accelerometer"),
                Gaussian(
                    (0.0, 0.0, 0.0),
                    0.0,
                    np.random.default_rng(secrets.randbits(64)),
                ),
                Gaussian(
                    (0.0, 0.0, 0.0),
                    0.0,
                    np.random.default_rng(secrets.randbits(64)),
                ),
            ),
            (moment,),
        ).frames(),
        equal_to(
            Aired(
                clean, Sensed(clean, "imu_gyro", "imu_accelerometer"), (moment,)
            ).frames()
        ),
        "board with exact sensors did not send the clean frames",
    )


def test_emulated_sends_the_same_noise_from_the_same_seed() -> None:
    """Two boards whose generators share a seed send identical frames."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    seed = secrets.randbits(64)
    moment = secrets.SystemRandom().uniform(0.05, 1.5)
    first = mujoco.MjData(model)
    second = mujoco.MjData(model)
    generators = (
        np.random.default_rng(seed),
        np.random.default_rng(seed),
    )
    assert_that(
        Aired(
            first,
            Noisy(
                Sensed(first, "imu_gyro", "imu_accelerometer"),
                Gaussian((0.008, -0.005, 0.007), 0.001, generators[0]),
                Gaussian((0.0, 0.0, 0.0), 0.02, generators[0]),
            ),
            (moment,),
        ).frames(),
        equal_to(
            Aired(
                second,
                Noisy(
                    Sensed(second, "imu_gyro", "imu_accelerometer"),
                    Gaussian((0.008, -0.005, 0.007), 0.001, generators[1]),
                    Gaussian((0.0, 0.0, 0.0), 0.02, generators[1]),
                ),
                (moment,),
            ).frames()
        ),
        "boards with one seed did not send the same frames",
    )


def test_emulated_sends_the_same_noise_however_the_clock_is_sliced() -> None:
    """With noise, one advance by a second still sends what a hundred send."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    seed = secrets.randbits(64)
    speed = secrets.SystemRandom().uniform(-6.0, 6.0)
    whole = mujoco.MjData(model)
    whole.ctrl = (-speed, speed)
    sliced = mujoco.MjData(model)
    sliced.ctrl = (-speed, speed)
    generators = (
        np.random.default_rng(seed),
        np.random.default_rng(seed),
    )
    assert_that(
        Aired(
            sliced,
            Noisy(
                Sensed(sliced, "imu_gyro", "imu_accelerometer"),
                Gaussian((-0.006, 0.009, 0.004), 0.001, generators[0]),
                Gaussian((0.0, 0.0, 0.0), 0.02, generators[0]),
            ),
            tuple(count * 0.01 for count in range(1, 101)),
        ).frames(),
        equal_to(
            Aired(
                whole,
                Noisy(
                    Sensed(whole, "imu_gyro", "imu_accelerometer"),
                    Gaussian((-0.006, 0.009, 0.004), 0.001, generators[1]),
                    Gaussian((0.0, 0.0, 0.0), 0.02, generators[1]),
                ),
                (1.0,),
            ).frames()
        ),
        "noisy board did not send the same frames when the clock was sliced",
    )
