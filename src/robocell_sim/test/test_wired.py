"""Tests for the harness whose topics a ROS 2 node wires up."""

import queue
import secrets

from geometry_msgs.msg import Twist, Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from hamcrest import assert_that, equal_to, has_entry
from rclpy.context import Context
from rclpy.node import Node
from robocell_sim.ros.implementations.wired import Wired


def test_wired_subscribes_to_the_velocity_command() -> None:
    """The harness subscribes to geometry_msgs/Twist on /cmd_vel."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"wired{secrets.token_hex(4)}", context=context)
    Wired(node).listen(queue.SimpleQueue())
    assert_that(
        dict(node.get_subscriber_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/cmd_vel", ["geometry_msgs/msg/Twist"]),
        "harness did not subscribe to the velocity command",
    )


def test_wired_queues_the_forward_speed_and_the_yaw_rate() -> None:
    """A Twist on /cmd_vel lands in the inbox as its linear x and angular z."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"wired{secrets.token_hex(4)}", context=context)
    inbox: queue.SimpleQueue[tuple[float, float]] = queue.SimpleQueue()
    Wired(node).listen(inbox)
    velocity = (
        secrets.SystemRandom().uniform(-0.26, 0.26),
        secrets.SystemRandom().uniform(-1.8, 1.8),
    )
    next(iter(node.subscriptions)).callback(
        Twist(
            linear=Vector3(x=velocity[0], y=0.1, z=0.2),
            angular=Vector3(x=0.3, y=0.4, z=velocity[1]),
        )
    )
    assert_that(
        inbox.get(timeout=1),
        equal_to(velocity),
        "harness did not queue the forward speed and the yaw rate",
    )


def test_wired_opens_the_joint_states_topic() -> None:
    """The harness advertises the wheel joints as sensor_msgs/JointState."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"wired{secrets.token_hex(4)}", context=context)
    Wired(node).joints()
    assert_that(
        dict(node.get_publisher_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/joint_states", ["sensor_msgs/msg/JointState"]),
        "harness did not advertise the joint states topic",
    )


def test_wired_opens_the_scan_topic() -> None:
    """The harness advertises the lidar as sensor_msgs/LaserScan on /scan."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"wired{secrets.token_hex(4)}", context=context)
    Wired(node).scan()
    assert_that(
        dict(node.get_publisher_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/scan", ["sensor_msgs/msg/LaserScan"]),
        "harness did not advertise the scan topic",
    )
