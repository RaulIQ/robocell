"""Tests for the outlet whose topic a ROS 2 node plugs in."""

import secrets

from hamcrest import assert_that, equal_to, has_entry
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy
from robocell_sim.ros.implementations.wiring.plugged import Plugged

from test.settled import Settled


def test_plugged_opens_the_imu_topic() -> None:
    """The outlet advertises the IMU readings as sensor_msgs/Imu on /imu."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"plugged{secrets.token_hex(4)}", context=context)
    Plugged(node).imu()
    assert_that(
        Settled(node, "/imu", 2.0).publishers(),
        has_entry("/imu", ["sensor_msgs/msg/Imu"]),
        "outlet did not advertise the imu topic",
    )


def test_plugged_sends_the_imu_best_effort() -> None:
    """The IMU readings go out best effort, so a lost one is never resent."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"plugged{secrets.token_hex(4)}", context=context)
    Plugged(node).imu()
    assert_that(
        Settled(node, "/imu", 2.0).publications()[0].qos_profile.reliability,
        equal_to(ReliabilityPolicy.BEST_EFFORT),
        "outlet did not send the imu best effort",
    )
