"""Tests for the slip that compares the encoder turn with the gyro turn."""

import math
import secrets
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.imu.integrated import Integrated
from robocell_sim.hardware.implementations.physics.paced import Paced
from robocell_sim.hardware.implementations.physics.physical import Physical
from robocell_sim.hardware.implementations.wheels.simulated import Simulated
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.compared import Compared

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.slip import Slip


def test_compared_trusts_a_turn_both_sensors_agree_on() -> None:
    """Encoders and gyro turning by the same angle report no slip."""
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Compared(
            Differential(0.033, 0.288),
            4096,
            0.3,
            ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)),
        )
        .read(
            -count,
            count,
            0.033 * 2 * count * 2 * math.pi / 4096 / 0.288,
            secrets.SystemRandom().uniform(0.01, 1.0),
        )
        .slipping(),
        equal_to(obj=False),
        "a turn both sensors agreed on did not pass as a firm grip",
    )


def test_compared_flags_a_turn_the_gyro_never_felt() -> None:
    """Encoders that turn while the gyro stays still report a slip."""
    count = secrets.SystemRandom().randint(1000, 50000)
    assert_that(
        Compared(
            Differential(0.033, 0.288),
            4096,
            0.3,
            ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)),
        )
        .read(-count, count, 0.0, secrets.SystemRandom().uniform(0.01, 0.1))
        .slipping(),
        equal_to(obj=True),
        "a turn the gyro never felt did not count as a slip",
    )


def test_compared_tolerates_a_gap_below_the_threshold() -> None:
    """A gap in turn rate below the threshold reports no slip."""
    threshold = secrets.SystemRandom().uniform(0.1, 1.0)
    moment = secrets.SystemRandom().uniform(0.01, 1.0)
    assert_that(
        Compared(
            Differential(0.033, 0.288),
            4096,
            threshold,
            ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)),
        )
        .read(0, 0, threshold * moment * 0.9, moment)
        .slipping(),
        equal_to(obj=False),
        "a gap below the threshold did not pass as a firm grip",
    )


def test_compared_judges_only_the_last_two_readings() -> None:
    """A slip in an earlier interval leaves a later agreeing one alone."""
    count = secrets.SystemRandom().randint(1000, 50000)
    assert_that(
        Compared(
            Differential(0.033, 0.288),
            4096,
            0.3,
            ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)),
        )
        .read(-count, count, 0.0, 0.05)
        .read(-count, count, 0.0, secrets.SystemRandom().uniform(0.06, 1.0))
        .slipping(),
        equal_to(obj=False),
        "an old slip did not leave the latest interval alone",
    )


def test_compared_finds_no_slip_before_any_time_passes() -> None:
    """Two readings at the same moment report no slip."""
    assert_that(
        Compared(
            Differential(0.033, 0.288),
            4096,
            0.3,
            ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)),
        ).slipping(),
        equal_to(obj=False),
        "readings at the same moment did not pass as a firm grip",
    )


def test_compared_flags_wheels_spinning_on_ice() -> None:
    """A robot spinning on a nearly frictionless floor reports a slip."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    model.geom_friction[:, 0] = 0.001
    data = mujoco.MjData(model)
    speed = secrets.SystemRandom().uniform(4.0, 7.88)
    data.ctrl = (-speed, speed)
    slip: Slip = Compared(
        Differential(0.033, 0.288),
        4096,
        0.3,
        ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)),
    )
    for moment in (1.0, 1.5):
        Paced(
            Integrated(Physical(model, data), data, "imu_gyro", 0), data
        ).advance(moment)
        slip = slip.read(
            Simulated(data, "wheel_left_encoder", 4096).ticks(),
            Simulated(data, "wheel_right_encoder", 4096).ticks(),
            float(data.userdata[0]),
            data.time,
        )
    assert_that(
        slip.slipping(),
        equal_to(obj=True),
        "wheels spinning on ice did not count as a slip",
    )


def test_compared_stays_quiet_on_a_dry_floor() -> None:
    """A robot driving an arc on the normal floor never reports a slip."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    data.ctrl = (
        secrets.SystemRandom().uniform(1.0, 3.0),
        secrets.SystemRandom().uniform(4.0, 6.0),
    )
    slip: Slip = Compared(
        Differential(0.033, 0.288),
        4096,
        0.3,
        ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)),
    )
    verdicts = []
    for sample in range(1, 31):
        Paced(
            Integrated(Physical(model, data), data, "imu_gyro", 0), data
        ).advance(sample * 0.1)
        slip = slip.read(
            Simulated(data, "wheel_left_encoder", 4096).ticks(),
            Simulated(data, "wheel_right_encoder", 4096).ticks(),
            float(data.userdata[0]),
            data.time,
        )
        verdicts.append(slip.slipping())
    assert_that(
        sum(verdicts),
        equal_to(0),
        "a drive on a dry floor did not stay free of slips",
    )
