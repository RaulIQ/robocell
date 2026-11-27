#!/usr/bin/env python3
"""Stationary gyro calibration between the raw IMU and the EKF."""

import contextlib
import logging
import math
import queue

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data

from robocell_sim.ros.implementations.clock.timed import Timed
from robocell_sim.ros.implementations.imu.calibrated import (
    Calibrated,
    String,
    zeroed,
)
from robocell_sim.ros.implementations.settings.declared import Declared
from robocell_sim.ros.interfaces.calibration import Imu, JointState

_MINIMUM = 2

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("gyro")
rclpy.init()
node = rclpy.create_node("gyro")
settings = Declared(node)
limits = (
    settings.number("duration", 2.0),
    settings.whole("samples", 100),
    settings.number("deviation", 0.003),
)
thresholds = (
    settings.number("wheel_limit", 0.02),
    settings.number("timeout", 0.1),
)
names = (
    settings.text("left_joint", "wheel_left_joint"),
    settings.text("right_joint", "wheel_right_joint"),
)
if (
    not all(
        math.isfinite(value) and value > 0 for value in (*limits, *thresholds)
    )
    or limits[1] < _MINIMUM
):
    error = "Gyro calibration requires positive limits and at least two samples"
    raise ValueError(error)
readings: queue.SimpleQueue[Imu] = queue.SimpleQueue()
joints: queue.SimpleQueue[JointState] = queue.SimpleQueue()
node.create_subscription(Imu, "/imu", readings.put, qos_profile_sensor_data)
node.create_subscription(JointState, "/joint_states", joints.put, 10)
topic = node.create_publisher(Imu, "/imu/calibrated", qos_profile_sensor_data)
status = node.create_publisher(
    String, "~/state", qos_profile_action_status_default
)
calibration = Calibrated(
    Timed(node),
    (limits, thresholds, names),
    (topic, status, log),
    [(zeroed(limits), "", -1, False)],
)
feedback = JointState()
status.publish(String(data="calibrating"))
log.info("Gyro calibration is waiting for a continuous stationary interval")
with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
    while node.context.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        while not joints.empty():
            feedback = joints.get()
        while not readings.empty():
            calibration.read(readings.get(), feedback)
node.context.try_shutdown()
