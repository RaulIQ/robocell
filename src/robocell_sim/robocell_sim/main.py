"""Entry point of robocell."""

import logging
import math
import multiprocessing
import queue
import threading
import time

import mujoco
import mujoco.viewer

from robocell_sim.implementations.accumulated import Accumulated
from robocell_sim.implementations.arrows import Arrows
from robocell_sim.implementations.compared import Compared
from robocell_sim.implementations.differential import Differential
from robocell_sim.implementations.gyroscopic import Gyroscopic
from robocell_sim.implementations.integrated import Integrated
from robocell_sim.implementations.limited import Limited
from robocell_sim.implementations.paced import Paced
from robocell_sim.implementations.physical import Physical
from robocell_sim.implementations.pinhole import Pinhole
from robocell_sim.implementations.polar import Polar
from robocell_sim.implementations.raycast import Raycast
from robocell_sim.implementations.screen import Screen
from robocell_sim.implementations.simulated import Simulated
from robocell_sim.implementations.stepped import Stepped
from robocell_sim.implementations.window import Window
from robocell_sim.interfaces.command import Command
from robocell_sim.interfaces.odometry import Odometry
from robocell_sim.interfaces.slip import Slip

model = mujoco.MjModel.from_xml_path(
    "/ws/src/robocell_sim/models/waffle_pi/workshop.xml"
)
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
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
gyro = Accumulated(data, 0)
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("robocell")
moment = 0.0
physics = Paced(Integrated(Physical(model, data), data, "imu_gyro", 0), data)
clock = 0.0
odometry: Odometry = Gyroscopic(
    drive,
    4096,
    (data.qpos[0], data.qpos[1], 2 * math.atan2(data.qpos[6], data.qpos[3])),
    (0, 0, 0.0),
)
slip: Slip = Compared(drive, 4096, 0.3, ((0, 0, 0.0, 0.0), (0, 0, 0.0, 0.0)))
route = Window(multiprocessing.get_context("fork").Queue())
multiprocessing.get_context("fork").Process(
    target=route.show, daemon=True
).start()
sample = 0.0
camera = Screen(
    multiprocessing.get_context("fork").Queue(maxsize=2),
    model,
    "camera_rgb",
    (640, 480),
)
multiprocessing.get_context("fork").Process(
    target=camera.show, daemon=True
).start()
log.info(
    "Camera sees through the intrinsics %s",
    Pinhole(
        math.radians(model.cam_fovy[model.camera("camera_rgb").id]), 640, 480
    ).matrix(),
)
shot = 0.0
lidar = Raycast(model, data, "base_scan", (0.12, 3.5))
radar = Polar(multiprocessing.get_context("fork").Queue(maxsize=2), 3.5)
multiprocessing.get_context("fork").Process(
    target=radar.show, daemon=True
).start()
scan = 0.0

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
            odometry = odometry.read(left.ticks(), right.ticks(), gyro.angle())
            if data.time >= sample:
                slipped = slip.slipping()
                slip = slip.read(
                    left.ticks(), right.ticks(), gyro.angle(), data.time
                )
                if slip.slipping() and not slipped:
                    log.warning(
                        "Wheels slip at %.1f s as the encoders and the gyro "
                        "disagree on the turn",
                        data.time,
                    )
                if slipped and not slip.slipping():
                    log.info("Wheels grip again at %.1f s", data.time)
                route.mark(
                    odometry.position(),
                    (
                        data.body("base_link").xpos[0],
                        data.body("base_link").xpos[1],
                    ),
                    slipping=slip.slipping(),
                )
                sample += 0.1
            if data.time >= scan:
                radar.mark(lidar.ranges())
                scan += 0.2
            if data.time >= shot:
                camera.record(data.qpos)
                shot += 1 / 30
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
