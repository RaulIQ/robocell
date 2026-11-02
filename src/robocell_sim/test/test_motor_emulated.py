"""Tests for CANopen drive endpoints with actual CAN transport and physics."""

import math
import struct
import time

import can
import mujoco
import pytest
from hamcrest import assert_that, close_to, equal_to

from test.energized import Energized
from test.motored import Motored


def test_motor_emulated_boots_two_independent_nodes() -> None:
    """Each configured node announces its own boot-up on the shared bus."""
    with Motored((17, 29)).connection() as (host, _, _, _):
        result = tuple(
            (message.arbitration_id, bytes(message.data))
            for message in iter(lambda: host.recv(0), None)
            if bytes(message.data) == b"\x00"
        )
    assert_that(
        result,
        equal_to(((1809, b"\x00"), (1821, b"\x00"))),
        "drives did not announce two independent boot-ups",
    )


@pytest.mark.parametrize(
    "speeds",
    [(1537, 1537), (-1731, 1731), (1231, -1231), (-2317, -2317), (0, 0)],
)
def test_motor_emulated_applies_signed_commands_only_after_enable(
    speeds: tuple[int, int],
) -> None:
    """CAN commands reach both actuators with the configured node identities."""
    with Energized((17, 29), speeds).connection() as (_, _, data, _):
        result = tuple(data.ctrl)
    assert_that(
        result,
        equal_to(tuple(speed / 1000 for speed in speeds)),
        "CAN targets did not reach both actuators",
    )


def test_motor_emulated_dont_move_a_disabled_drive() -> None:
    """NMT start is communication permission rather than motor permission."""
    with Motored((17, 29)).connection() as (host, devices, data, _):
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x01\x00", is_extended_id=False
            )
        )
        host.send(
            can.Message(
                arbitration_id=529,
                data=struct.pack("<Hi", 15, 2713),
                is_extended_id=False,
            )
        )
        devices[0].advance(time.monotonic())
        result = float(data.ctrl[0])
    assert_that(result, equal_to(0.0), "disabled drive did not reject motion")


def test_motor_emulated_answers_standard_sdo_reads() -> None:
    """The independent host reads the profile through the library SDO server."""
    with Motored((17, 29)).connection() as (host, devices, _, _):
        tuple(iter(lambda: host.recv(0), None))
        host.send(
            can.Message(
                arbitration_id=1553,
                data=bytes.fromhex("40 03 20 00 00 00 00 00"),
                is_extended_id=False,
            )
        )
        devices[0].advance(time.monotonic())
        result = bytes(next(iter(lambda: host.recv(0.1), None)).data)
    assert_that(
        result,
        equal_to(bytes.fromhex("43 03 20 00 01 00 00 00")),
        "SDO server did not return the profile revision",
    )


def test_motor_emulated_watchdog_stops_the_physical_actuator() -> None:
    """Missing command frames zero the actual MuJoCo control boundary."""
    with Energized((17, 29), (1937, 2371)).connection() as (
        _,
        devices,
        data,
        _,
    ):
        devices[0].advance(time.monotonic() + 0.201)
        result = float(data.ctrl[0])
    assert_that(
        result, equal_to(0.0), "watchdog did not stop the physical actuator"
    )


def test_motor_emulated_heartbeat_loss_cannot_be_hidden_by_commands() -> None:
    """A command stream alone cannot keep a disconnected host alive."""
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        _,
    ):
        moment = time.monotonic()
        host.send(
            can.Message(
                arbitration_id=529,
                data=struct.pack("<Hi", 15, 1937),
                is_extended_id=False,
            )
        )
        devices[0].advance(moment + 0.15)
        devices[0].advance(moment + 0.301)
        result = float(data.ctrl[0])
    assert_that(
        result,
        equal_to(0.0),
        "heartbeat supervision did not inhibit the actuator",
    )


def test_motor_emulated_quick_stop_reaches_the_actuator() -> None:
    """Quick-stop affects the shaft before waiting for feedback."""
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        _,
    ):
        host.send(
            can.Message(
                arbitration_id=529,
                data=struct.pack("<Hi", 2, 0),
                is_extended_id=False,
            )
        )
        devices[0].advance(time.monotonic())
        result = float(data.ctrl[0])
    assert_that(
        result, equal_to(0.0), "quick-stop did not inhibit the actuator"
    )


def test_motor_emulated_pause_cannot_replay_a_cached_target() -> None:
    """Resume remains disabled until the host performs a new enable walk."""
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        pause,
    ):
        pause.set()
        devices[0].advance(time.monotonic())
        pause.clear()
        host.send(
            can.Message(
                arbitration_id=529,
                data=struct.pack("<Hi", 15, 1937),
                is_extended_id=False,
            )
        )
        devices[0].advance(time.monotonic())
        result = float(data.ctrl[0])
    assert_that(
        result, equal_to(0.0), "resume did not discard the cached velocity"
    )


def test_motor_emulated_nmt_stop_inhibits_the_actuator() -> None:
    """Leaving operational communication cannot leave a target applied."""
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        _,
    ):
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x02\x11", is_extended_id=False
            )
        )
        devices[0].advance(time.monotonic())
        result = float(data.ctrl[0])
    assert_that(result, equal_to(0.0), "NMT stop did not inhibit the actuator")


def test_motor_emulated_reset_requires_new_enable_permission() -> None:
    """A drive restart discards the previous target."""
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        _,
    ):
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x81\x11", is_extended_id=False
            )
        )
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x01\x11", is_extended_id=False
            )
        )
        host.send(
            can.Message(
                arbitration_id=529,
                data=struct.pack("<Hi", 15, 1937),
                is_extended_id=False,
            )
        )
        devices[0].advance(time.monotonic())
        result = float(data.ctrl[0])
    assert_that(
        result, equal_to(0.0), "reset did not discard the previous velocity"
    )


def test_motor_emulated_feedback_comes_from_physics() -> None:
    """Feedback carries actual motion and simulated acquisition time."""
    with Motored((17, 29)).connection() as (host, devices, data, _):
        tuple(iter(lambda: host.recv(0), None))
        data.qpos[0] = -0.1
        data.qvel[0] = -0.7334
        data.time = 0.123456789
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x01\x11", is_extended_id=False
            )
        )
        devices[0].advance(time.monotonic())
        result = tuple(
            (message.arbitration_id, bytes(message.data))
            for message in iter(lambda: host.recv(0), None)
        )
    assert_that(
        result,
        equal_to(
            (
                (401, struct.pack("<HiH", 64, -66, 0)),
                (657, struct.pack("<iH", -733, 0)),
                (913, struct.pack("<Q", 123456789)),
            )
        ),
        "PDOs did not carry actual physics measurements",
    )


def test_motor_emulated_moves_the_physical_wheel() -> None:
    """Accepted targets cause actual joint motion."""
    with Energized((17, 29), (-1937, 2371)).connection() as (_, _, data, _):
        mujoco.mj_step(data.model, data, nstep=30)
        result = float(data.qvel[0])
    assert_that(
        result,
        close_to(-1.937, 0.05),
        "CAN command did not move the MuJoCo wheel",
    )


@pytest.mark.parametrize(
    ("payload", "abort"),
    [
        ("40 34 12 00 00 00 00 00", 100794368),
        ("40 41 60 01 00 00 00 00", 101253137),
        ("23 FF 60 00 E8 03 00 00", 100728834),
        ("21 60 60 00 01 00 00 00", 84148225),
        ("2F 60 60 00 09 00 00 00", 101253168),
        ("2B 60 60 00 03 00 00 00", 101122064),
    ],
)
def test_motor_emulated_rejects_unsupported_sdo_access(
    payload: str, abort: int
) -> None:
    """Unsupported access produces a precise SDO abort."""
    with Motored((17, 29)).connection() as (host, devices, _, _):
        tuple(iter(lambda: host.recv(0), None))
        host.send(
            can.Message(
                arbitration_id=1553,
                data=bytes.fromhex(payload),
                is_extended_id=False,
            )
        )
        devices[0].advance(time.monotonic())
        result = bytes(next(iter(lambda: host.recv(0.1), None)).data)[4:]
    assert_that(
        result,
        equal_to(struct.pack("<I", abort)),
        "SDO server did not reject unsupported access",
    )


@pytest.mark.parametrize(
    ("payload", "extended", "fd", "remote"),
    [
        (b"\x0f\x00", False, False, False),
        (struct.pack("<Hi", 15, 3217), True, False, False),
        (struct.pack("<Hi", 15, 3217), False, True, False),
        (b"", False, False, True),
    ],
)
def test_motor_emulated_dont_renew_commands_with_malformed_frames(
    payload: bytes, *, extended: bool, fd: bool, remote: bool
) -> None:
    """Invalid wire frames cannot keep a motor running past its deadline."""
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        _,
    ):
        moment = time.monotonic()
        host.send(
            can.Message(
                arbitration_id=529,
                data=payload,
                is_extended_id=extended,
                is_fd=fd,
                is_remote_frame=remote,
            )
        )
        devices[0].advance(moment + 0.15)
        devices[0].advance(moment + 0.201)
        result = float(data.ctrl[0])
    assert_that(
        result,
        equal_to(0.0),
        "malformed frame did not leave the watchdog deadline intact",
    )


def test_motor_emulated_shutdown_zeroes_both_actuators() -> None:
    """Closing emulator contexts inhibits output even without a host stop."""
    with Energized((17, 29), (1937, 2371)).connection() as (_, _, data, _):
        pass
    assert_that(
        tuple(data.ctrl),
        equal_to((0.0, 0.0)),
        "shutdown did not inhibit both actuators",
    )


def test_motor_emulated_dont_republish_feedback_without_a_physics_step() -> (
    None
):
    """Repeated service at one acquisition time cannot invent samples."""
    interval = (0x180, 0x400)
    with Motored((17, 29)).connection() as (host, devices, _, _):
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x01\x11", is_extended_id=False
            )
        )
        devices[0].advance(time.monotonic())
        tuple(iter(lambda: host.recv(0), None))
        devices[0].advance(time.monotonic())
        result = tuple(
            message.arbitration_id
            for message in iter(lambda: host.recv(0), None)
            if interval[0] <= message.arbitration_id < interval[1]
        )
    assert_that(
        result,
        equal_to(()),
        "stationary simulation time did not suppress duplicate feedback",
    )


def test_motor_emulated_wraps_the_encoder_without_resetting_physics() -> None:
    """The signed wire counter wraps without resetting the physical angle."""
    identifier = 0x191
    with Motored((17, 29)).connection() as (host, devices, data, _):
        data.qpos[0] = 2**31 * 2 * math.pi / 4096
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x01\x11", is_extended_id=False
            )
        )
        devices[0].advance(time.monotonic())
        result = tuple(
            struct.unpack("<HiH", message.data)[1]
            for message in iter(lambda: host.recv(0), None)
            if message.arbitration_id == identifier
        )
    assert_that(
        result, equal_to((-(2**31),)), "encoder did not wrap its signed counter"
    )


def test_motor_emulated_clock_reset_reboots_the_disabled_drive() -> None:
    """A backwards clock starts a disabled epoch."""
    identifier = 0x711
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        _,
    ):
        data.time = 0.147
        devices[0].advance(time.monotonic())
        tuple(iter(lambda: host.recv(0), None))
        data.time = 0.0
        devices[0].advance(time.monotonic())
        result = (
            float(data.ctrl[0]),
            tuple(
                bytes(message.data)
                for message in iter(lambda: host.recv(0), None)
                if message.arbitration_id == identifier
                and bytes(message.data) == b"\0"
            ),
        )
    assert_that(
        result,
        equal_to((0.0, (b"\0",))),
        "clock reset did not inhibit and reboot the drive",
    )


def test_motor_emulated_recovers_only_through_reset_and_enable() -> None:
    """A recovered link needs fault reset and fresh enable commands."""
    enable = 15
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        data,
        _,
    ):
        moment = time.monotonic() + 0.201
        devices[0].advance(moment)
        host.send(
            can.Message(
                arbitration_id=0x77F, data=b"\x05", is_extended_id=False
            )
        )
        for word in (128, 0, 6, 7, 15, 15):
            host.send(
                can.Message(
                    arbitration_id=0x211,
                    data=struct.pack(
                        "<Hi", word, -1327 if word == enable else 0
                    ),
                    is_extended_id=False,
                )
            )
        devices[0].advance(moment + 0.002)
        result = float(data.ctrl[0])
    assert_that(
        result,
        equal_to(-1.327),
        "recovered drive did not accept a fresh enabled command",
    )


def test_motor_emulated_cannot_clear_a_missing_host_with_nmt_reset() -> None:
    """NMT reset cannot clear an active heartbeat fault."""
    identifier = 0x591
    with Energized((17, 29), (1937, 2371)).connection() as (
        host,
        devices,
        _,
        _,
    ):
        moment = time.monotonic() + 0.301
        devices[0].advance(moment)
        host.send(
            can.Message(
                arbitration_id=0, data=b"\x81\x11", is_extended_id=False
            )
        )
        host.send(
            can.Message(
                arbitration_id=0x611,
                data=bytes.fromhex("40 3F 60 00 00 00 00 00"),
                is_extended_id=False,
            )
        )
        devices[0].advance(moment + 0.002)
        result = tuple(
            bytes(message.data)[4:6]
            for message in iter(lambda: host.recv(0), None)
            if message.arbitration_id == identifier
        )
    assert_that(
        result,
        equal_to((bytes.fromhex("30 81"),)),
        "NMT reset did not preserve the active heartbeat fault",
    )
