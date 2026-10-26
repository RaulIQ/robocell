#!/usr/bin/env python3
"""Entry point of robocell."""

import logging
import math
import multiprocessing
import queue
import threading
import time

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory

from robocell_sim.display.implementations.beams import Beams
from robocell_sim.display.implementations.dots import Dots
from robocell_sim.display.implementations.polar import Polar
from robocell_sim.display.implementations.scanned import Scanned
from robocell_sim.display.implementations.screen import Screen
from robocell_sim.display.implementations.survey import Survey
from robocell_sim.display.implementations.window import Window
from robocell_sim.hardware.implementations.accumulated import Accumulated
from robocell_sim.hardware.implementations.integrated import Integrated
from robocell_sim.hardware.implementations.paced import Paced
from robocell_sim.hardware.implementations.physical import Physical
from robocell_sim.hardware.implementations.pinhole import Pinhole
from robocell_sim.hardware.implementations.raycast import Raycast
from robocell_sim.hardware.implementations.simulated import Simulated
from robocell_sim.robot.implementations.arrows import Arrows
from robocell_sim.robot.implementations.compared import Compared
from robocell_sim.robot.implementations.differential import Differential
from robocell_sim.robot.implementations.guarded import Guarded
from robocell_sim.robot.implementations.gyroscopic import Gyroscopic
from robocell_sim.robot.implementations.limited import Limited
from robocell_sim.robot.implementations.placed import Placed
from robocell_sim.robot.implementations.stepped import Stepped
from robocell_sim.robot.interfaces.command import Command
from robocell_sim.robot.interfaces.odometry import Odometry
from robocell_sim.robot.interfaces.slip import Slip
from robocell_sim.ros.implementations.advertised import Advertised
from robocell_sim.ros.implementations.clocked import Clocked
from robocell_sim.ros.implementations.grounded import Grounded
from robocell_sim.ros.implementations.jointed import Jointed
from robocell_sim.ros.implementations.periodic import Periodic
from robocell_sim.ros.implementations.ranged import Ranged
from robocell_sim.ros.implementations.tracked import Tracked
from robocell_sim.ros.implementations.wired import Wired

model = mujoco.MjModel.from_xml_path(
    f"{get_package_share_directory('robocell_sim')}/models/waffle_pi/workshop.xml"
)
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
pause = threading.Event()
beams = threading.Event()
keys: queue.SimpleQueue[int] = queue.SimpleQueue()
twists: queue.SimpleQueue[tuple[float, float]] = queue.SimpleQueue()


def key(keycode: int) -> None:
    """Toggle the pause on space and the lidar lines on 9, queue other keys."""
    toggle = {ord(" "): pause, ord("9"): beams}.get(keycode)
    if toggle is None:
        keys.put(keycode)
        return
    if toggle.is_set():
        toggle.clear()
    else:
        toggle.set()


drive = Limited(Differential(0.033, 0.288), 7.88)
command: Command = Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0))
left = Simulated(data, "wheel_left_encoder", 4096)
right = Simulated(data, "wheel_right_encoder", 4096)
gyro = Accumulated(data, 0)
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("robocell")
moment = 0.0
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
reach = (0.12, 3.5)
lidar = Raycast(model, data, "base_scan", reach)
radar = Polar(multiprocessing.get_context("fork").Queue(maxsize=2), 3.5)
multiprocessing.get_context("fork").Process(
    target=radar.show, daemon=True
).start()
scan = 0.0
atlas = Survey(multiprocessing.get_context("fork").Queue())
multiprocessing.get_context("fork").Process(
    target=atlas.show, daemon=True
).start()
survey = 0.0
ranges = tuple(math.inf for _ in range(360))

rclpy.init()
node = rclpy.create_node("sim")
bridge = Advertised(node)
harness = Wired(node)
harness.listen(twists)
physics = Paced(
    Periodic(
        Periodic(
            Periodic(
                Periodic(
                    Periodic(
                        Integrated(Physical(model, data), data, "imu_gyro", 0),
                        data,
                        1 / 100,
                        Clocked(data, bridge.clock()),
                    ),
                    data,
                    1 / 50,
                    Tracked(data, "base_link", bridge.truth()),
                ),
                data,
                1 / 50,
                Grounded(data, "base_link", bridge.frames()),
            ),
            data,
            1 / 50,
            Jointed(
                data,
                {"wheel_left_joint": left, "wheel_right_joint": right},
                harness.joints(),
            ),
        ),
        data,
        0.2,
        Ranged(data, lidar, reach, harness.scan()),
    ),
    data,
)

with mujoco.viewer.launch_passive(model, data, key_callback=key) as viewer:
    viewer.set_texts(
        (None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT.value, "9", "Lidar lines")
    )
    frame = time.time()
    while viewer.is_running():
        now = time.time()
        rclpy.spin_once(node, timeout_sec=0)
        while not keys.empty():
            command = command.press(keys.get())
        while not twists.empty():
            command = command.steer(*twists.get())
        if not pause.is_set():
            data.ctrl[
                [
                    model.actuator("wheel_left").id,
                    model.actuator("wheel_right").id,
                ]
            ] = drive.wheels(*Guarded(command, ranges, 0.3, 0.2).velocity())
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
                ranges = lidar.ranges()
                radar.mark(ranges)
                if data.time >= survey:
                    atlas.mark(
                        Placed(
                            ranges,
                            (
                                data.body("base_link").xpos[0],
                                data.body("base_link").xpos[1],
                                math.atan2(
                                    data.body("base_link").xmat[3],
                                    data.body("base_link").xmat[0],
                                ),
                            ),
                            -0.064,
                        ).points(),
                        Placed(
                            ranges,
                            (*odometry.position(), odometry.heading()),
                            -0.064,
                        ).points(),
                    )
                    survey += 1.0
                with viewer.lock():
                    viewer.user_scn.ngeom = 0
                    if beams.is_set():
                        Beams(Scanned(data, "base_scan", ranges)).paint(
                            viewer.user_scn
                        )
                    Dots(Scanned(data, "base_scan", ranges)).paint(
                        viewer.user_scn
                    )
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

rclpy.shutdown()
