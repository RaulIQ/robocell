"""Tests for the emulated IMU board that sends its readings on CAN."""

import itertools
import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.packed import ACCELEROMETER, GYRO

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
    assert_that(
        [
            identifier
            for identifier, _ in Aired(
                mujoco.MjData(
                    mujoco.MjModel.from_xml_path(
                        str(
                            Path(__file__).parents[1]
                            / "models"
                            / "waffle_pi"
                            / "scene.xml"
                        )
                    )
                ),
                (1.0,),
            ).frames()
        ],
        equal_to([GYRO, ACCELEROMETER] * 100),
        "board did not send one gyro and one accelerometer frame per period",
    )


def test_emulated_numbers_each_measurement_one_past_the_last() -> None:
    """The measurement number grows by one every period of simulated time."""
    assert_that(
        {
            (later[6] - earlier[6]) % 256
            for earlier, later in itertools.pairwise(
                payload
                for identifier, payload in Aired(
                    mujoco.MjData(
                        mujoco.MjModel.from_xml_path(
                            str(
                                Path(__file__).parents[1]
                                / "models"
                                / "waffle_pi"
                                / "scene.xml"
                            )
                        )
                    ),
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
    assert_that(
        {
            (earlier[6], later[6])
            for earlier, later in itertools.pairwise(
                payload
                for identifier, payload in Aired(
                    mujoco.MjData(
                        mujoco.MjModel.from_xml_path(
                            str(
                                Path(__file__).parents[1]
                                / "models"
                                / "waffle_pi"
                                / "scene.xml"
                            )
                        )
                    ),
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
        Aired(sliced, tuple(count * 0.01 for count in range(1, 101))).frames(),
        equal_to(Aired(whole, (1.0,)).frames()),
        "board did not send the same frames when the clock advanced in slices",
    )
