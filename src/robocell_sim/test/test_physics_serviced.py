"""Protective CAN service brackets every actual physical step."""

import threading

import mujoco
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.physics.physical import Physical
from robocell_sim.hardware.implementations.physics.serviced import Serviced

from test.noted import Noted


def test_physics_serviced_checks_protection_before_and_after_a_step() -> None:
    """CAN service sees both sides of a physics boundary."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_string(
            '<mujoco><option timestep="0.0037"/></mujoco>'
        )
    )
    moments: list[float] = []
    Serviced(
        Physical(data.model, data), Noted(moments, data), threading.Event()
    ).take()
    assert_that(
        moments,
        equal_to([0.0, 0.0037]),
        "protective service did not bracket the physics step",
    )


def test_physics_serviced_keeps_servicing_can_while_paused() -> None:
    """Pause inhibits stepping while preserving protective CAN checks."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_string(
            '<mujoco><option timestep="0.0037"/></mujoco>'
        )
    )
    moments: list[float] = []
    pause = threading.Event()
    pause.set()
    Serviced(Physical(data.model, data), Noted(moments, data), pause).take()
    assert_that(
        moments,
        equal_to([0.0, 0.0]),
        "pause did not preserve protective service without advancing physics",
    )
