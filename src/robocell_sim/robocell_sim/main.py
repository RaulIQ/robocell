"""Entry point of robocell."""

import logging
import queue
import threading
import time

import mujoco
import mujoco.viewer

from robocell_sim.implementations.arrows import Arrows
from robocell_sim.implementations.differential import Differential
from robocell_sim.implementations.limited import Limited
from robocell_sim.implementations.paced import Paced
from robocell_sim.implementations.simulated import Simulated
from robocell_sim.implementations.stepped import Stepped
from robocell_sim.interfaces.command import Command

model = mujoco.MjModel.from_xml_path(
    "/ws/src/robocell_sim/models/waffle_pi/scene.xml"
)
data = mujoco.MjData(model)
pause = threading.Event()
keys: queue.SimpleQueue[int] = queue.SimpleQueue()


def key(keycode: int) -> None:
    """Toggle the pause on space and queue every other key for the drive."""
    if keycode != ord(" "):
        keys.put(keycode)
        return
    if pause.is_set():
        pause.clear()
    else:
        pause.set()


drive = Limited(Differential(0.033, 0.288), 7.88)
command: Command = Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0))
left = Simulated(data, "wheel_left_encoder", 4096)
right = Simulated(data, "wheel_right_encoder", 4096)
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("robocell")
moment = 0.0
physics = Paced(model, data)
clock = 0.0

with mujoco.viewer.launch_passive(model, data, key_callback=key) as viewer:
    frame = time.time()
    while viewer.is_running():
        now = time.time()
        while not keys.empty():
            command = command.press(keys.get())
        if not pause.is_set():
            data.ctrl[
                [
                    model.actuator("wheel_left").id,
                    model.actuator("wheel_right").id,
                ]
            ] = drive.wheels(*command.velocity())
            clock += now - frame
            physics.advance(clock)
            if data.time >= moment:
                log.info(
                    "Encoders read %d ticks on the left and %d on the right",
                    left.ticks(),
                    right.ticks(),
                )
                moment += 0.5
        frame = now
        viewer.sync()
        time.sleep(max(0.0, model.opt.timestep - (time.time() - now)))
