"""Both configured drives enter operational together before PDO pairing."""

import secrets

import can
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.motor.assembled import Assembled
from robocell_sim.robot.implementations.motor.negotiated import Negotiated
from robocell_sim.robot.implementations.motor.observed import Observed
from robocell_sim.robot.implementations.motor.serviced import Serviced
from robocell_sim.robot.implementations.motor.started import Started
from robocell_sim.robot.implementations.motor.unwrapped import Unwrapped


def test_motor_serviced_waits_for_both_configurations_before_nmt_start() -> (
    None
):
    """A faster drive cannot start its feedback cadence ahead of its peer."""
    channel = secrets.token_hex(12)
    session = Started((17, 29), ()).session(1.0, recovery=False)
    axes = tuple(
        Observed(
            Negotiated(
                number,
                ((0, b"\x01\x11", b""),)
                if offset == 0
                else (
                    (
                        0x61D,
                        bytes.fromhex("40 03 20 00 00 00 00 00"),
                        bytes.fromhex("43 03 20 00 01 00 00 00"),
                    ),
                ),
                (-1.0, -1.0, 0),
                (127, 1.0),
            ),
            Assembled((), 1.0),
            Unwrapped((0, 0, -1), _valid=False),
            (),
        )
        for offset, number in enumerate((17, 29))
    )
    with (
        can.Bus(interface="virtual", channel=channel) as bus,
        can.Bus(interface="virtual", channel=channel) as observer,
    ):
        Serviced(
            bus, (17, 29), (axes, *session[1:]), (0.033, 0.288, 7.88, 0.002)
        ).start(1.0, 0)
        result = tuple(
            message.arbitration_id
            for message in iter(lambda: observer.recv(0), None)
        )
    assert_that(
        result,
        equal_to((0x61D,)),
        "fast drive did not wait for its peer before NMT start",
    )
