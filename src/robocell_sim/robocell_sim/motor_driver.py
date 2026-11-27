#!/usr/bin/env python3
"""Standalone ROS host of the synthetic CANopen wheel drives."""

import contextlib
import logging
import queue
import time
from contextlib import ExitStack

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_action_status_default
from sensor_msgs.msg import JointState  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.implementations.command.arrows import Arrows
from robocell_sim.robot.implementations.command.filtered import Filtered
from robocell_sim.robot.implementations.drive.stepped import Stepped
from robocell_sim.robot.implementations.motor.hosted import Hosted
from robocell_sim.robot.interfaces.control import Control
from robocell_sim.robot.interfaces.pilot import Pilot, Session
from robocell_sim.ros.implementations.clock.received import Clock, Received
from robocell_sim.ros.implementations.command.steered import Steered
from robocell_sim.ros.implementations.motor.progressed import Progressed, String
from robocell_sim.ros.implementations.motor.reconnected import Reconnected
from robocell_sim.ros.implementations.motor.retry import Retry, Trigger
from robocell_sim.ros.implementations.settings.declared import Declared

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("motor_driver")
rclpy.init()
node = rclpy.create_node("motor_driver")
settings = Declared(node)
transport = (
    settings.text("interface", "socketcan"),
    settings.text("channel", "vcan0"),
)
numbers = (settings.whole("left_node", 1), settings.whole("right_node", 2))
geometry = (
    settings.number("wheel_radius", 0.033),
    settings.number("wheel_separation", 0.288),
    settings.number("wheel_limit", 7.88),
    settings.number("physics_step", 0.002),
)
names = (
    settings.text("left_joint", "wheel_left_joint"),
    settings.text("right_joint", "wheel_right_joint"),
)
state: queue.SimpleQueue[Session] = queue.SimpleQueue()
controls: queue.SimpleQueue[Control] = queue.SimpleQueue()
controls.put(
    Filtered(
        Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0)),
        -1.0,
    )
)
steering = Steered(node, controls)
steering.listen()
retries: queue.SimpleQueue[bool] = queue.SimpleQueue()
clocks: queue.SimpleQueue[int] = queue.SimpleQueue()
pilots: list[Pilot] = []
retries.put(item=False)


node.create_subscription(Clock, "/clock", Received(clocks), 10)
node.create_service(Trigger, "~/retry", Retry(retries, _recovery=False))
node.create_service(Trigger, "~/recover", Retry(retries, _recovery=True))
topic = node.create_publisher(JointState, "/joint_states", 10)
announcements = node.create_publisher(
    String,
    "~/state",
    qos_profile_action_status_default,
)
publication = [("", -1)]
progressed = Progressed(names, (topic, announcements), log, publication)
simulation = -1
with (
    contextlib.suppress(KeyboardInterrupt, ExternalShutdownException),
    ExitStack() as resources,
):
    connect = Reconnected(
        (Hosted(transport, numbers, geometry, state), resources),
        retries,
        [],
        (log, transport[1], numbers),
    )
    while node.context.ok():
        beginning = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0)
        if connect(beginning, pilots):
            publication[0] = "starting", -1
            announcements.publish(String(data="starting"))
        connect.connect(beginning, pilots)
        if (
            not pilots
            and connect.expired(beginning)
            and publication[0][0] == "starting"
        ):
            publication[0] = "stopped", publication[0][1]
            announcements.publish(String(data="stopped"))
            log.error(
                "CANopen connection exceeded five seconds and requires ~/retry"
            )
        while not clocks.empty():
            simulation = clocks.get_nowait()
        linear, angular, moment = steering.reading()
        if pilots:
            try:
                pilots[0].command((linear, angular), moment)
            except ValueError:
                log.exception("CANopen input rejected a nonfinite command")
        if pilots:
            progressed(pilots[0], time.monotonic(), simulation)
        time.sleep(max(0.0, 0.005 - (time.monotonic() - beginning)))
node.context.try_shutdown()
