#!/usr/bin/env python3
"""Publish fresh ArUco target observations in the navigation frame."""

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped, Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from message_filters import ApproximateTimeSynchronizer, Subscriber  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import ColorRGBA, String  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_geometry_msgs import do_transform_pose_stamped  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from tf2_ros import (  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
    Buffer,
    ExtrapolationException,
    TransformException,
    TransformListener,
)
from visualization_msgs.msg import Marker  # type: ignore[import-untyped]  # Jazzy ships this module without typing information

from robocell_sim.projection import MissingDepth
from robocell_sim.ros.implementations.pose.observed import Observed
from robocell_sim.ros.implementations.settings.declared import Declared

rclpy.init()
node = rclpy.create_node("aruco")
settings = Declared(node)
identifier = settings.whole("marker_id", 0)
if not 0 <= identifier < 50:
    raise ValueError("ArUco marker ID must be between 0 and 49")
frame = settings.text("target_frame", "odom")
bridge = CvBridge()
observed = Observed(identifier, cv2.aruco.DICT_4X4_50, bridge)
output = node.create_publisher(PoseStamped, "/target/pose", 1)
annotated = node.create_publisher(
    Image, "/camera/aruco/image_raw", qos_profile_sensor_data
)
points = node.create_publisher(Marker, "/target/marker", 1)
status = node.create_publisher(
    String,
    "/target/status",
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)
state = ["waiting"]
contact = [0]


def report(current: str) -> None:
    """Publish and log each target-state transition once."""
    if state[0] == current:
        return
    state[0] = current
    status.publish(String(data=current))
    node.get_logger().info(f"Target {identifier} {current}")
    if current != "detected":
        points.publish(Marker(ns="camera_target", action=Marker.DELETE))


def unavailable(error: TransformException) -> None:
    """Report a real TF failure with its diagnostic detail."""
    if state[0] != "tf_unavailable":
        node.get_logger().warning(f"Target {identifier} TF unavailable: {error}")
    report("tf_unavailable")


def expire() -> None:
    """Mark a target stale when valid transformed observations stop arriving."""
    if (
        state[0] in ("detected", "tf_unavailable")
        and node.get_clock().now().nanoseconds - contact[0] > 500_000_000
    ):
        report("stale")


def observe(image: Image, depth: Image, calibration: CameraInfo) -> None:
    """Publish only fresh observations with a transform at the image time."""
    stamp = Time.from_msg(image.header.stamp)
    if not (0 <= (node.get_clock().now() - stamp).nanoseconds <= 500_000_000):
        report("stale")
        annotated.publish(image)
        return
    try:
        detection = observed.detect(image, depth, calibration)
        observation = detection.observation()
        if not observation.valid():
            raise ValueError("Configured marker was not detected")
    except MissingDepth:
        report("invalid_depth")
        annotated.publish(image)
        return
    except ValueError:
        report("lost")
        annotated.publish(image)
        return
    preview = bridge.cv2_to_imgmsg(detection.image(), "rgb8")
    preview.header = image.header
    annotated.publish(preview)
    position = observation.position()
    goal = PoseStamped()
    goal.header.stamp = image.header.stamp
    goal.header.frame_id = observation.frame()
    goal.pose.position = Point(x=position[0], y=position[1], z=position[2])
    goal.pose.orientation.w = 1.0
    try:
        transform = buffer.lookup_transform(frame, observation.frame(), stamp)
    except ExtrapolationException as error:
        try:
            latest = buffer.lookup_transform(frame, observation.frame(), Time())
        except TransformException:
            unavailable(error)
        else:
            if Time.from_msg(latest.header.stamp) >= stamp:
                unavailable(error)
        return
    except TransformException as error:
        unavailable(error)
        return
    target = do_transform_pose_stamped(goal, transform)
    output.publish(target)
    contact[0] = node.get_clock().now().nanoseconds
    points.publish(
        Marker(
            header=target.header,
            ns="camera_target",
            type=Marker.SPHERE,
            pose=target.pose,
            scale=Vector3(x=0.12, y=0.12, z=0.12),
            color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
        )
    )
    report("detected")


images = Subscriber(
    node, Image, "/camera/image_raw", qos_profile=qos_profile_sensor_data
)
depths = Subscriber(
    node,
    Image,
    "/camera/depth/registered/image_raw",
    qos_profile=qos_profile_sensor_data,
)
calibrations = Subscriber(
    node, CameraInfo, "/camera/camera_info", qos_profile=qos_profile_sensor_data
)
synchronizer = ApproximateTimeSynchronizer([images, depths, calibrations], 5, 0.02)
synchronizer.registerCallback(observe)
status.publish(String(data="waiting"))
node.create_timer(0.1, expire)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
