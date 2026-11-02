"""Tests for the drive's physical MuJoCo boundary."""

import threading

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.motor.actuated import Actuated


def test_actuated_applies_the_velocity_to_its_own_wheel() -> None:
    """The right shaft's command cannot drive the left actuator."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><body><joint name="wheel"/>'
            '<geom size="0.1"/></body></worldbody>'
            '<actuator><velocity name="motor" joint="wheel"/></actuator>'
            "</mujoco>"
        )
    )
    Actuated(data, "motor", "wheel", threading.Event()).spin(-2.347)
    assert_that(
        float(data.ctrl[0]), equal_to(-2.347), "shaft did not apply its target"
    )


def test_actuated_reports_measured_velocity_instead_of_the_target() -> None:
    """An unmoving wheel cannot report its command as actual speed."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><body><joint name="wheel"/>'
            '<geom size="0.1"/></body></worldbody>'
            '<actuator><velocity name="motor" joint="wheel"/></actuator>'
            "</mujoco>"
        )
    )
    shaft = Actuated(data, "motor", "wheel", threading.Event())
    shaft.spin(3.173)
    assert_that(
        shaft.measurement()[1],
        close_to(0.0, 1e-9),
        "shaft did not report actual motion",
    )
