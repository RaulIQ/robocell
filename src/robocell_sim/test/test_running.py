"""Workshop physics continue protective CAN service throughout a pause."""

import threading

import mujoco
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.motor.polled import Polled
from robocell_sim.hardware.implementations.physics.physical import Physical
from robocell_sim.hardware.implementations.physics.running import Running
from robocell_sim.hardware.implementations.physics.serviced import Serviced

from test.delayed import Delayed
from test.energized import Energized
from test.noted import Noted


def test_running_zeroes_real_actuators_before_waiting_on_pause() -> None:
    """An already moving pair of drives is inhibited without advancing time."""
    with Energized((17, 29), (1937, -2371)).connection() as (
        _,
        devices,
        data,
        pause,
    ):
        pause.set()
        report = Polled(devices)
        Running(
            Serviced(Physical(data.model, data), report, pause),
            data,
            pause,
            report,
        ).advance(13.71)
        result = (*tuple(data.ctrl), float(data.time))
    assert_that(
        result,
        equal_to((0.0, 0.0, 0.0)),
        "paused workshop did not inhibit both actuators before waiting",
    )


def test_running_returns_to_the_viewer_after_a_slow_step() -> None:
    """Catch-up cannot monopolize the thread that updates the window."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_string(
            '<mujoco><option timestep="0.002"/></mujoco>'
        )
    )
    times: list[float] = []
    Running(
        Delayed(Physical(data.model, data), 0.031),
        data,
        threading.Event(),
        Noted(times, data),
    ).advance(0.017)
    assert_that(
        times,
        equal_to([0.0, 0.002]),
        "slow catch-up did not yield after one expensive physical step",
    )


def test_running_continues_unfinished_physics_on_the_next_frame() -> None:
    """Yielding preserves every physical step and resumes on the next call."""
    data = mujoco.MjData(
        mujoco.MjModel.from_xml_string(
            '<mujoco><option timestep="0.002"/></mujoco>'
        )
    )
    times: list[float] = []
    physics = Running(
        Delayed(Physical(data.model, data), 0.029),
        data,
        threading.Event(),
        Noted(times, data),
    )
    for _ in range(3):
        physics.advance(0.006)
    assert_that(
        times,
        equal_to([0.0, 0.002, 0.002, 0.004, 0.004, 0.006]),
        "yielding did not preserve and resume unfinished physical steps",
    )
