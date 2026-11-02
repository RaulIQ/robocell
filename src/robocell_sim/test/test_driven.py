"""The host drives independent CAN endpoints and stops real actuators."""

import time

import can
import pytest
from hamcrest import assert_that, close_to, equal_to, starts_with

from test.piloted import Piloted


@pytest.mark.parametrize(
    ("linear", "angular"),
    [(0.061, -0.137), (0.123, 0.0), (-0.073, 0.191), (0.0, -0.317)],
)
def test_driven_enables_both_wheels_before_accepting_fresh_motion(
    linear: float, angular: float
) -> None:
    """The real handshake and PDO path reach both physical wheel targets."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        _,
        data,
        _,
        _,
    ):
        flight.enable()
        flight.advance(0.05, ((linear, angular),))
        result = float(data.ctrl[0])
    assert_that(
        result,
        close_to((linear - angular * 0.288 / 2) / 0.033, 0.00051),
        "host did not command the enabled left wheel through CAN",
    )


def test_driven_cannot_replay_a_command_received_before_enable() -> None:
    """Initialization invalidates commands buffered during startup."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        pilot,
        data,
        _,
        _,
    ):
        pilot.command((0.123, -0.317), time.monotonic())
        flight.enable()
        flight.advance(0.06, ())
        result = tuple(data.ctrl)
    assert_that(
        result, equal_to((0.0, 0.0)), "startup did not discard cached motion"
    )


def test_driven_expires_stale_body_commands_at_both_actuators() -> None:
    """Continued PDO renewal cannot conceal an abandoned body command."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        _,
        data,
        _,
        _,
    ):
        flight.enable()
        flight.advance(0.05, ((0.073, -0.137),))
        flight.advance(0.29, ())
        result = tuple(data.ctrl)
    assert_that(
        result, equal_to((0.0, 0.0)), "stale input did not stop both actuators"
    )


def test_driven_rejects_a_latched_fault_without_operator_recovery() -> None:
    """A normal retry never clears a drive's command-watchdog fault."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        pilot,
        _,
        _,
        _,
    ):
        flight.enable()
        time.sleep(0.22)
        flight.step(())
        pilot.retry(time.monotonic(), recovery=False)
        result = flight.enable()
    assert_that(
        result,
        starts_with("stopped:"),
        "normal retry did not preserve the fault",
    )


def test_driven_recovers_only_with_explicit_fault_permission() -> None:
    """A watchdog fault clears through explicit reset and fresh enable."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        pilot,
        data,
        _,
        _,
    ):
        flight.enable()
        time.sleep(0.22)
        flight.step(())
        pilot.retry(time.monotonic(), recovery=True)
        flight.enable()
        flight.advance(0.06, ((-0.073, 0.0),))
        result = float(data.ctrl[0])
    assert_that(
        result,
        close_to(-0.073 / 0.033, 0.00051),
        "explicit recovery did not allow fresh motion",
    )


def test_driven_stops_the_peer_when_one_drive_leaves_operational() -> None:
    """Non-operational heartbeat cannot leave the reachable peer moving."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        _,
        data,
        bus,
        _,
    ):
        flight.enable()
        flight.advance(0.05, ((0.073, -0.137),))
        bus.send(
            can.Message(
                arbitration_id=0, data=b"\x02\x11", is_extended_id=False
            )
        )
        flight.advance(0.15, ((0.073, -0.137),))
        result = tuple(data.ctrl)
    assert_that(
        result,
        equal_to((0.0, 0.0)),
        "lost drive did not stop its reachable peer",
    )


def test_driven_cannot_resume_motion_after_invalid_input() -> None:
    """Rejecting a nonfinite body command invalidates earlier motion."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        pilot,
        data,
        _,
        _,
    ):
        flight.enable()
        flight.advance(0.05, ((0.073, 0.137),))
        with pytest.raises(ValueError, match="nonfinite"):
            pilot.command((float("nan"), 0.137), time.monotonic())
        flight.advance(0.04, ())
        result = tuple(data.ctrl)
    assert_that(
        result,
        equal_to((0.0, 0.0)),
        "invalid command did not invalidate previous motion",
    )


def test_driven_publishes_measured_velocity_from_physics() -> None:
    """Paired feedback contains actual shaft velocity, not requested speed."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        _,
        data,
        _,
        _,
    ):
        flight.enable()
        data.qvel[1] = -1.937
        data.time += 0.021
        _, feedback = flight.step(())
    assert_that(
        feedback[0][2][1],
        equal_to(-1.937),
        "CAN feedback did not report the physical right wheel speed",
    )


def test_driven_stops_when_acquisition_disappears_but_clock_advances() -> None:
    """Heartbeat alone cannot conceal absent physical measurements."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        _,
        _,
        _,
        pause,
    ):
        flight.enable()
        pause.set()
        status, _ = flight.advance(0.55, ())
    assert_that(
        status,
        starts_with("stopped:"),
        "advancing clock did not expose missing feedback",
    )


def test_driven_cannot_replay_motion_after_pause_and_resume() -> None:
    """Resuming physics repeats verified enable and waits for fresh input."""
    with Piloted((17, 29), _external=True).connection() as (
        flight,
        _,
        data,
        _,
        pause,
    ):
        flight.enable()
        flight.advance(0.04, ((0.073, -0.137),))
        pause.set()
        flight.advance(0.04, ())
        pause.clear()
        flight.enable()
        result = tuple(data.ctrl)
    assert_that(
        result, equal_to((0.0, 0.0)), "resume did not discard pre-pause motion"
    )


def test_driven_supervises_feedback_without_an_external_clock() -> None:
    """Absent PDOs cannot imply a paused simulator without a clock source."""
    with Piloted((17, 29), _external=False).connection() as (
        flight,
        pilot,
        _,
        _,
        pause,
    ):
        flight.enable()
        flight.advance(0.04, ((0.073, -0.137),))
        pause.set()
        flight.step(())
        time.sleep(0.31)
        pilot.command((0.073, -0.137), time.monotonic())
        status, _ = pilot.advance(time.monotonic(), -1)
    assert_that(
        status,
        starts_with("stopped: Drive feedback exceeded"),
        "missing feedback without an external clock did not stop the driver",
    )
