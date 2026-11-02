"""Workshop physics continue protective CAN service throughout a pause."""

from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.motor.polled import Polled
from robocell_sim.hardware.implementations.physics.physical import Physical
from robocell_sim.hardware.implementations.physics.running import Running
from robocell_sim.hardware.implementations.physics.serviced import Serviced

from test.energized import Energized


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
