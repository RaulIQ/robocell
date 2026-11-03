#!/usr/bin/env python3
"""Standalone ROS host of the synthetic CANopen wheel drives."""

import contextlib
import logging
import queue
import time
from contextlib import ExitStack

import can
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_action_status_default
from rosgraph_msgs.msg import Clock  # type: ignore[import-untyped]  # ROS generates message modules without types
from sensor_msgs.msg import JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import String  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_srvs.srv import Trigger  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.implementations.command.arrows import Arrows
from robocell_sim.robot.implementations.command.filtered import Filtered
from robocell_sim.robot.implementations.drive.stepped import Stepped
from robocell_sim.robot.implementations.motor.hosted import Hosted
from robocell_sim.robot.interfaces.control import Control
from robocell_sim.robot.interfaces.pilot import Pilot, Session
from robocell_sim.ros.implementations.command.steered import Steered
from robocell_sim.ros.implementations.motor.reported import Reported
from robocell_sim.ros.implementations.settings.declared import Declared

_BACKOFF = 0.1

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
        ((), -1),
        (0.3, 0.2, 500000000),
    )
)
steering = Steered(
    node, controls, _protected=settings.flag("collision_guard", default=False)
)
steering.listen()
retries: queue.SimpleQueue[bool] = queue.SimpleQueue()
clocks: queue.SimpleQueue[int] = queue.SimpleQueue()
pilots: list[Pilot] = []
retries.put(item=False)


def clock(message: Clock) -> None:
    """Observe the simulator clock independently of drive PDO arrival."""
    clocks.put(message.clock.sec * 1000000000 + message.clock.nanosec)


def retry(
    _request: Trigger.Request, response: Trigger.Response
) -> Trigger.Response:
    """Request a verified enable without granting fault-reset permission."""
    retries.put(item=False)
    response.success = True
    response.message = "CANopen retry requested without fault reset"
    return response


def recover(
    _request: Trigger.Request, response: Trigger.Response
) -> Trigger.Response:
    """Record explicit operator permission for one fault-recovery attempt."""
    retries.put(item=True)
    response.success = True
    response.message = "CANopen recovery requested with fault-reset permission"
    return response


node.create_subscription(Clock, "/clock", clock, 10)
node.create_service(Trigger, "~/retry", retry)
node.create_service(Trigger, "~/recover", recover)
topic = node.create_publisher(JointState, "/joint_states", 10)
announcements = node.create_publisher(
    String,
    "~/state",
    qos_profile_action_status_default,
)
status = ""
stamp = -1
simulation = -1
attempt = -1.0
deadline = -1.0
permission = False
with (
    contextlib.suppress(KeyboardInterrupt, ExternalShutdownException),
    ExitStack() as resources,
):
    while node.context.ok():
        beginning = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0)
        if not retries.empty():
            resources.close()
            pilots.clear()
            permission = retries.get_nowait()
            deadline = beginning + 5.0
            attempt = -1.0
            status = "starting"
            stamp = -1
            announcements.publish(String(data="starting"))
        if (
            not pilots
            and beginning < deadline
            and (attempt < 0 or beginning - attempt >= _BACKOFF)
        ):
            attempt = beginning
            try:
                pilots.append(
                    resources.enter_context(
                        Hosted(transport, numbers, geometry, state).connection(
                            deadline - 5.0, recovery=permission
                        )
                    )
                )
                log.info(
                    "CANopen host on %s is verifying drives %d and %d",
                    transport[1],
                    *numbers,
                )
            except (can.CanError, OSError) as failure:
                log.warning(
                    "CANopen connection on %s failed because %s",
                    transport[1],
                    failure,
                )
        if not pilots and beginning >= deadline and status == "starting":
            status = "stopped"
            announcements.publish(String(data=status))
            log.error(
                "CANopen connection exceeded five seconds and requires ~/retry"
            )
        while not clocks.empty():
            simulation = clocks.get_nowait()
        linear, angular, moment = steering.reading(simulation)
        if pilots:
            try:
                pilots[0].command((linear, angular), moment)
            except ValueError:
                log.exception("CANopen input rejected a nonfinite command")
        if pilots:
            condition, measurements = pilots[0].advance(
                time.monotonic(), simulation
            )
            if condition != status:
                log.info("CANopen host is %s", condition)
                status = condition
                announcements.publish(String(data=condition))
            for sample in measurements:
                if sample[0] != stamp:
                    Reported(sample, names, topic).publish()
                    stamp = sample[0]
        time.sleep(max(0.0, 0.005 - (time.monotonic() - beginning)))
node.context.try_shutdown()
