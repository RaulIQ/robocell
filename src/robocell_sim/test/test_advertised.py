"""Tests for the bridge whose topics a ROS 2 node advertises."""

import secrets

from hamcrest import assert_that, has_entry
from rclpy.context import Context
from rclpy.node import Node
from robocell_sim.ros.implementations.advertised import Advertised


def test_advertised_opens_the_clock_topic() -> None:
    """The bridge advertises the clock as rosgraph_msgs/Clock on /clock."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"advertised{secrets.token_hex(4)}", context=context)
    Advertised(node).clock()
    assert_that(
        dict(node.get_publisher_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/clock", ["rosgraph_msgs/msg/Clock"]),
        "bridge did not advertise the clock topic",
    )


def test_advertised_opens_the_ground_truth_topic() -> None:
    """The bridge advertises the truth as nav_msgs/Odometry."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"advertised{secrets.token_hex(4)}", context=context)
    Advertised(node).truth()
    assert_that(
        dict(node.get_publisher_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/ground_truth/odom", ["nav_msgs/msg/Odometry"]),
        "bridge did not advertise the ground truth topic",
    )


def test_advertised_opens_the_transform_topic() -> None:
    """The bridge advertises the frames as tf2_msgs/TFMessage on /tf."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"advertised{secrets.token_hex(4)}", context=context)
    Advertised(node).frames()
    assert_that(
        dict(node.get_publisher_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/tf", ["tf2_msgs/msg/TFMessage"]),
        "bridge did not advertise the transform topic",
    )
