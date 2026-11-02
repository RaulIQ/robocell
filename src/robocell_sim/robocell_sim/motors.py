#!/usr/bin/env python3
"""Run two CANopen wheel emulators against MuJoCo without ROS."""

import threading
import time
from argparse import ArgumentParser
from contextlib import ExitStack
from pathlib import Path

import can
import mujoco
import mujoco.viewer

from robocell_sim.hardware.implementations.motor.actuated import Actuated
from robocell_sim.hardware.implementations.motor.attached import Attached
from robocell_sim.hardware.implementations.motor.stepped import Stepped

_HOST = 127

arguments = ArgumentParser(description=__doc__)
arguments.add_argument("--model", type=Path, required=True)
arguments.add_argument("--interface", default="socketcan")
arguments.add_argument("--channel", default="vcan0")
arguments.add_argument("--left-node", type=int, default=1)
arguments.add_argument("--right-node", type=int, default=2)
arguments.add_argument(
    "--duration",
    type=float,
    default=0.0,
    help="Wall duration in seconds; zero runs until interrupted",
)
arguments.add_argument("--viewer", action="store_true")
options = arguments.parse_args()
if options.left_node == options.right_node or not all(
    1 <= number < _HOST for number in (options.left_node, options.right_node)
):
    arguments.error("Wheel nodes must be distinct and between 1 and 126")
if options.duration < 0:
    arguments.error("Duration must be nonnegative")
model = mujoco.MjModel.from_xml_path(str(options.model))
data = mujoco.MjData(model)
if model.nkey:
    mujoco.mj_resetDataKeyframe(model, data, 0)
pause = threading.Event()


def key(code: int) -> None:
    """Toggle physics pause with the space bar."""
    if code == ord(" "):
        if pause.is_set():
            pause.clear()
        else:
            pause.set()


with ExitStack() as stack:
    devices = tuple(
        stack.enter_context(
            Attached(
                stack.enter_context(
                    can.Bus(
                        interface=options.interface, channel=options.channel
                    )
                ),
                number,
                Actuated(data, f"wheel_{side}", f"wheel_{side}_joint", pause),
            ).connection()
        )
        for side, number in (
            ("left", options.left_node),
            ("right", options.right_node),
        )
    )

    def running() -> bool:
        """Keep the headless service alive until interrupted or timed out."""
        return True

    if options.viewer:
        viewer = stack.enter_context(
            mujoco.viewer.launch_passive(model, data, key_callback=key)
        )
        running = viewer.is_running
    step = Stepped(
        data, devices, Actuated(data, "wheel_left", "wheel_left_joint", pause)
    )
    beginning = time.monotonic()
    try:
        while running() and (
            options.duration == 0
            or time.monotonic() - beginning < options.duration
        ):
            step.take()
            if options.viewer:
                viewer.sync()
    except KeyboardInterrupt:
        pass
