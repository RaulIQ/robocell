#!/usr/bin/env python3
"""Stationary gyro calibration between the raw IMU and the EKF."""

import contextlib
import logging
import math
import queue

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import String  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.implementations.imu.averaged import Averaged
from robocell_sim.robot.implementations.imu.zeroed import Zeroed
from robocell_sim.robot.interfaces.zero import Zero
from robocell_sim.ros.implementations.imu.corrected import Corrected
from robocell_sim.ros.implementations.imu.rested import Rested
from robocell_sim.ros.implementations.settings.declared import Declared

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
zero: Zero = Zeroed(limits, (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3)))
feedback = JointState()
frame = ""
stamp = -1
ready = False
status.publish(String(data="calibrating"))
log.info("Gyro calibration is waiting for a continuous stationary interval")
with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
    while node.context.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        while not joints.empty():
            feedback = joints.get()
        while not readings.empty():
            message = readings.get()
            moment = Time.from_msg(message.header.stamp).nanoseconds
            if moment == stamp:
                continue
            if (
                abs(node.get_clock().now().nanoseconds - moment) * 1e-9
                > thresholds[1]
                or message.angular_velocity_covariance[0] < 0
            ):
                if not zero.ready():
                    zero = Zeroed(
                        limits,
                        (0.0, 0.0),
                        Averaged((0, (0.0,) * 3, (0.0,) * 3)),
                    )
                continue
            if frame != message.header.frame_id:
                zero = Zeroed(
                    limits, (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
                )
                frame = message.header.frame_id
            try:
                zero = zero.read(
                    (
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z,
                    ),
                    moment * 1e-9,
                    still=Rested(feedback, message, thresholds, names).still(),
                )
            except ValueError:
                zero = Zeroed(
                    limits, (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
                )
                log.warning(
                    "Gyro calibration rejected a nonfinite IMU measurement"
                )
            stamp = moment
            if zero.ready() != ready:
                ready = zero.ready()
                status.publish(String(data="ready" if ready else "calibrating"))
                if ready:
                    log.info(
                        "Gyro offsets are %.6f, %.6f and %.6f rad/s",
                        *zero.bias(),
                    )
                else:
                    log.info(
                        "Gyro calibration requires a new stationary interval"
                    )
            if zero.ready():
                Corrected(message, zero, topic).publish()
node.context.try_shutdown()
