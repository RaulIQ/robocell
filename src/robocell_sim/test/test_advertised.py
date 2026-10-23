"""Tests for the bridge whose topics a ROS 2 node advertises."""

import queue
import secrets

from geometry_msgs.msg import Twist, Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from hamcrest import assert_that, equal_to, has_entry
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


def test_advertised_subscribes_to_the_velocity_command() -> None:
    """The bridge subscribes to geometry_msgs/Twist on /cmd_vel."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"advertised{secrets.token_hex(4)}", context=context)
    Advertised(node).listen(queue.SimpleQueue())
    assert_that(
        dict(node.get_subscriber_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/cmd_vel", ["geometry_msgs/msg/Twist"]),
        "bridge did not subscribe to the velocity command",
    )


def test_advertised_queues_the_forward_speed_and_the_yaw_rate() -> None:
    """A Twist on /cmd_vel lands in the inbox as its linear x and angular z."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"advertised{secrets.token_hex(4)}", context=context)
    inbox: queue.SimpleQueue[tuple[float, float]] = queue.SimpleQueue()
    Advertised(node).listen(inbox)
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
        "bridge did not queue the forward speed and the yaw rate",
    )


def test_advertised_opens_the_joint_states_topic() -> None:
    """The bridge advertises the wheel joints as sensor_msgs/JointState."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"advertised{secrets.token_hex(4)}", context=context)
    Advertised(node).joints()
    assert_that(
        dict(node.get_publisher_names_and_types_by_node(node.get_name(), "/")),
        has_entry("/joint_states", ["sensor_msgs/msg/JointState"]),
        "bridge did not advertise the joint states topic",
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
