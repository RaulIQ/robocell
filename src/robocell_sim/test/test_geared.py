"""Tests for the instruments whose topics a ROS 2 node gears up."""

import math
import queue
import secrets

from builtin_interfaces.msg import Time  # type: ignore[import-untyped]  # ROS generates message modules without types
from hamcrest import assert_that, equal_to, has_entry
from rclpy.context import Context
from rclpy.node import Node
from robocell_sim.ros.implementations.wiring.geared import Geared
from sensor_msgs.msg import JointState  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.settled import Settled


def test_geared_opens_the_pose_topic() -> None:
    """The instruments advertise the wheel odometry on /wheel/odometry."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"geared{secrets.token_hex(4)}", context=context)
    Geared(node, ("left", "right"), 4096, "odom").pose()
    assert_that(
        Settled(node, "/wheel/odometry", 2.0).publishers(),
        has_entry("/wheel/odometry", ["nav_msgs/msg/Odometry"]),
        "instruments did not advertise the pose topic",
    )


def test_geared_opens_the_transforms_topic() -> None:
    """The instruments advertise the transforms as tf2_msgs/TFMessage."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"geared{secrets.token_hex(4)}", context=context)
    Geared(node, ("left", "right"), 4096, "odom").transforms()
    assert_that(
        Settled(node, "/tf", 2.0).publishers(),
        has_entry("/tf", ["tf2_msgs/msg/TFMessage"]),
        "instruments did not advertise the transforms topic",
    )


def test_geared_subscribes_to_the_joint_states() -> None:
    """The instruments subscribe to sensor_msgs/JointState on /joint_states."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"geared{secrets.token_hex(4)}", context=context)
    Geared(node, ("left", "right"), 4096, "odom").count(queue.SimpleQueue())
    assert_that(
        Settled(node, "/joint_states", 2.0).subscribers(),
        has_entry("/joint_states", ["sensor_msgs/msg/JointState"]),
        "instruments did not subscribe to the joint states",
    )


def test_geared_counts_the_left_wheel_by_its_joint() -> None:
    """The left count comes from the joint named left, wherever it sits."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"geared{secrets.token_hex(4)}", context=context)
    inbox: queue.SimpleQueue[tuple[int, int, float, Header]] = (
        queue.SimpleQueue()
    )
    resolution = secrets.SystemRandom().randint(100, 10_000)
    Geared(node, ("wheel_ł", "wheel_ř"), resolution, "odom").count(inbox)
    turns = secrets.SystemRandom().randint(-50, 50)
    next(iter(node.subscriptions)).callback(
        JointState(
            name=["caster", "wheel_ř", "wheel_ł"],
            position=[0.4, -1.1, turns * 2 * math.pi / resolution],
        )
    )
    assert_that(
        inbox.get(timeout=1)[0],
        equal_to(turns),
        "instruments did not count the left wheel by its joint",
    )


def test_geared_counts_the_right_wheel_by_its_joint() -> None:
    """The right count comes from the joint named right, wherever it sits."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"geared{secrets.token_hex(4)}", context=context)
    inbox: queue.SimpleQueue[tuple[int, int, float, Header]] = (
        queue.SimpleQueue()
    )
    resolution = secrets.SystemRandom().randint(100, 10_000)
    Geared(node, ("wheel_ł", "wheel_ř"), resolution, "odom").count(inbox)
    turns = secrets.SystemRandom().randint(-50, 50)
    next(iter(node.subscriptions)).callback(
        JointState(
            name=["wheel_ř", "caster", "wheel_ł"],
            position=[turns * 2 * math.pi / resolution, 0.4, -1.1],
        )
    )
    assert_that(
        inbox.get(timeout=1)[1],
        equal_to(turns),
        "instruments did not count the right wheel by its joint",
    )


def test_geared_stamps_the_counts_in_the_odometry_frame() -> None:
    """The counts carry the joint stamp in the frame the odometry lies in."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"geared{secrets.token_hex(4)}", context=context)
    inbox: queue.SimpleQueue[tuple[int, int, float, Header]] = (
        queue.SimpleQueue()
    )
    frame = f"ódom_{secrets.token_hex(3)}"
    Geared(node, ("left", "right"), 4096, frame).count(inbox)
    stamp = Time(
        sec=secrets.SystemRandom().randint(0, 2**30),
        nanosec=secrets.SystemRandom().randint(0, 999_999_999),
    )
    next(iter(node.subscriptions)).callback(
        JointState(
            header=Header(stamp=stamp, frame_id="base_link"),
            name=["left", "right"],
            position=[0.2, -0.9],
        )
    )
    assert_that(
        inbox.get(timeout=1)[3],
        equal_to(Header(stamp=stamp, frame_id=frame)),
        "instruments did not stamp the counts in the odometry frame",
    )


def test_geared_times_the_counts_by_the_joint_stamp() -> None:
    """The counts carry the stamp of the joint states in seconds."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    node = Node(f"geared{secrets.token_hex(4)}", context=context)
    inbox: queue.SimpleQueue[tuple[int, int, float, Header]] = (
        queue.SimpleQueue()
    )
    Geared(node, ("left", "right"), 4096, "odom").count(inbox)
    stamp = (
        secrets.SystemRandom().randint(0, 2**30),
        secrets.SystemRandom().randint(0, 999_999_999),
    )
    next(iter(node.subscriptions)).callback(
        JointState(
            header=Header(stamp=Time(sec=stamp[0], nanosec=stamp[1])),
            name=["left", "right"],
            position=[0.2, -0.9],
        )
    )
    assert_that(
        inbox.get(timeout=1)[2],
        equal_to((stamp[0] * 10**9 + stamp[1]) * 1e-9),
        "instruments did not time the counts by the joint stamp",
    )
