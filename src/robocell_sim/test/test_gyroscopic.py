"""Tests for the odometry that turns by the gyro and moves by the encoders."""

import math
import secrets
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
from hamcrest import assert_that, close_to, equal_to, less_than
from robocell_sim.hardware.implementations.accumulated import Accumulated
from robocell_sim.hardware.implementations.integrated import Integrated
from robocell_sim.hardware.implementations.paced import Paced
from robocell_sim.hardware.implementations.physical import Physical
from robocell_sim.hardware.implementations.simulated import Simulated
from robocell_sim.robot.implementations.differential import Differential
from robocell_sim.robot.implementations.gyroscopic import Gyroscopic

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.odometry import Odometry


def test_gyroscopic_drives_straight_by_the_rim_distance() -> None:
    """Equal counts and a still gyro move the robot by the rim distance."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    resolution = secrets.SystemRandom().randint(1024, 8192)
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Gyroscopic(
            Differential(radius, 0.288),
            resolution,
            (0.0, 0.0, 0.0),
            (0, 0, 0.0),
        )
        .read(count, count, 0.0)
        .position()[0],
        close_to(count * 2 * math.pi / resolution * radius, 1e-9),
        "straight counts did not move the robot by the rim distance",
    )


def test_gyroscopic_ignores_the_turn_of_the_encoders() -> None:
    """Uneven counts with a still gyro keep the robot on its heading."""
    assert_that(
        Gyroscopic(
            Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0, 0.0)
        )
        .read(0, secrets.SystemRandom().randint(1, 50000), 0.0)
        .position()[1],
        equal_to(0.0),
        "uneven counts did not keep the robot on the heading of the gyro",
    )


def test_gyroscopic_moves_along_the_turn_of_the_gyro() -> None:
    """A run during a gyro turn goes along the middle of that turn."""
    angle = secrets.SystemRandom().uniform(-math.pi, math.pi)
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Gyroscopic(
            Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0, 0.0)
        )
        .read(count, count, angle)
        .position()[1],
        close_to(
            count * 2 * math.pi / 4096 * 0.033 * math.sin(angle / 2), 1e-9
        ),
        "a run during a gyro turn did not go along the middle of the turn",
    )


def test_gyroscopic_counts_from_the_last_reading() -> None:
    """Reading the same counts and angle again leaves the position as it was."""
    left = secrets.SystemRandom().randint(-50000, 50000)
    right = secrets.SystemRandom().randint(-50000, 50000)
    angle = secrets.SystemRandom().uniform(-50.0, 50.0)
    odometry = Gyroscopic(
        Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0, 0.0)
    ).read(left, right, angle)
    assert_that(
        odometry.read(left, right, angle).position(),
        equal_to(odometry.position()),
        "same counts and angle did not leave the position where it was",
    )


def test_gyroscopic_follows_the_simulated_waffle_pi() -> None:
    """Gyro odometry of a driving robot ends within a centimeter of it."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    data.ctrl = (
        secrets.SystemRandom().uniform(1.0, 3.0),
        secrets.SystemRandom().uniform(4.0, 6.0),
    )
    odometry: Odometry = Gyroscopic(
        Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0, 0.0)
    )
    for frame in range(1, 301):
        Paced(
            Integrated(Physical(model, data), data, "imu_gyro", 0), data
        ).advance(frame * 0.01)
        odometry = odometry.read(
            Simulated(data, "wheel_left_encoder", 4096).ticks(),
            Simulated(data, "wheel_right_encoder", 4096).ticks(),
            Accumulated(data, 0).angle(),
        )
    assert_that(
        math.dist(odometry.position(), data.body("base_link").xpos[:2]),
        less_than(0.01),
        "gyro odometry did not stay within a centimeter of the robot",
    )
