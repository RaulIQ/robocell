#!/usr/bin/env python3
"""Driver that reads the IMU board over CAN and publishes /imu."""

import contextlib
import logging
import sys

import can
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.implementations.can.drained import Drained
from robocell_sim.robot.implementations.imu.logged import Logged
from robocell_sim.robot.implementations.imu.received import FILTERS, Received
from robocell_sim.robot.implementations.imu.scaled import Scaled
from robocell_sim.robot.implementations.imu.silent import Silent
from robocell_sim.robot.interfaces.pairing import Pairing
from robocell_sim.ros.implementations.imu.inertial import Inertial

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger("imu")
rclpy.init()
node = rclpy.create_node("imu")
channel = str(node.declare_parameter("channel", "vcan0").value)
period = float(node.declare_parameter("period", 0.005).value)
frame = str(node.declare_parameter("frame_id", "imu_link").value)
variances = (
    float(node.declare_parameter("gyro_variance", 1e-6).value),
    float(node.declare_parameter("accelerometer_variance", 4e-4).value),
)
topic = node.create_publisher(Imu, "/imu", qos_profile_sensor_data)
try:
    bus = can.Bus(interface="socketcan", channel=channel, can_filters=FILTERS)
except OSError:
    log.critical(
        "CAN interface %s is missing, so bring it up on the host with "
        "sudo systemctl start vcan0",
        channel,
    )
    sys.exit(1)
inbox = Drained(bus)
journal = Logged(log)
pairing: Pairing = Silent(0)
with contextlib.suppress(KeyboardInterrupt, ExternalShutdownException):
    while node.context.ok():
        rclpy.spin_once(node, timeout_sec=period)
        for identifier, payload in inbox.frames():
            pairing = pairing.read(Received(identifier, payload))
            pairing.note(journal)
            if pairing.complete():
                Inertial(
                    Scaled(pairing),
                    Header(
                        stamp=node.get_clock().now().to_msg(), frame_id=frame
                    ),
                    variances,
                    topic,
                ).publish()
bus.shutdown()
node.context.try_shutdown()
