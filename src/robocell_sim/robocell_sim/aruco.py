#!/usr/bin/env python3
"""Publish fresh ArUco target observations in the navigation frame."""

from collections.abc import Callable

import cv2
import numpy as np
import numpy.typing as npt
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import try_shutdown
from std_msgs.msg import ColorRGBA, String  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_geometry_msgs import do_transform_pose_stamped  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from visualization_msgs.msg import Marker  # type: ignore[import-untyped]  # Jazzy ships this module without typing information

from robocell_sim.detection import Detected
from robocell_sim.freshness import fresh
from robocell_sim.missing_depth import MissingDepthError
from robocell_sim.observation import Observation
from robocell_sim.projection import Projected
from robocell_sim.ros.implementations.pose.observed import Observed
from robocell_sim.ros.implementations.settings.declared import Declared
from robocell_sim.ros.messages import (
    Buffer,
    CameraInfo,
    ExtrapolationException,
    Image,
    Point,
    PoseStamped,
    TransformException,
    TransformListener,
    Vector3,
)

_MARKERS = 50
_TIMEOUT = 500_000_000

rclpy.init()
node = rclpy.create_node("aruco")
node_logger: Callable[[], RcutilsLogger] = node.get_logger
logger = node_logger()
info: Callable[[str], None] = logger.info
warning: Callable[[str], None] = logger.warning
destroy: Callable[[], None] = node.destroy_node
settings = Declared(node)
identifier = settings.whole("marker_id", 0)
if not 0 <= identifier < _MARKERS:
    error = "ArUco marker ID must be between 0 and 49"
    raise ValueError(error)
frame = settings.text("target_frame", "odom")
bridge = CvBridge()
observed = Observed(
    Detected(cv2.aruco.DICT_4X4_50, identifier, Projected((3, 3))), bridge
)
output = node.create_publisher(PoseStamped, "/target/pose", 1)
annotated = node.create_publisher(
    Image, "/camera/aruco/image_raw", qos_profile_sensor_data
)
points = node.create_publisher(Marker, "/target/marker", 1)
policy: Callable[..., QoSProfile] = QoSProfile
status = node.create_publisher(
    String,
    "/target/status",
    policy(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
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
    info(f"Target {identifier} {current}")
    if current != "detected":
        points.publish(Marker(ns="camera_target", action=Marker.DELETE))


def unavailable(error: TransformException) -> None:
    """Report a real TF failure with its diagnostic detail."""
    if state[0] != "tf_unavailable":
        warning(f"Target {identifier} TF unavailable: {error}")
    report("tf_unavailable")


def expire() -> None:
    """Mark a target stale when valid transformed observations stop arriving."""
    elapsed = node.get_clock().now().nanoseconds - contact[0]
    if state[0] in {"detected", "tf_unavailable"} and elapsed > _TIMEOUT:
        report("stale")


def reject(image: Image, reason: str) -> None:
    """Publish the original image and report why it gave no usable target."""
    report(reason)
    annotated.publish(image)


def observe(image: Image, depth: Image, calibration: CameraInfo) -> None:
    """Detect the configured marker only in a fresh synchronized frame."""
    if current(image):
        measure(image, depth, calibration)
    else:
        reject(image, "stale")


def current(image: Image) -> bool:
    """Check the image timestamp against the simulation clock."""
    stamp = Time.from_msg(image.header.stamp).nanoseconds / 1e9
    return fresh(stamp, node.get_clock().now().nanoseconds / 1e9)


def measure(image: Image, depth: Image, calibration: CameraInfo) -> None:
    """Read the marker and publish a preview even when the reading fails."""
    try:
        observation, preview = observed(image, depth, calibration)
    except MissingDepthError:
        reject(image, "invalid_depth")
    except ValueError:
        reject(image, "lost")
    else:
        track(image, observation, preview)


def track(
    image: Image, observation: Observation, pixels: npt.NDArray[np.uint8]
) -> None:
    """Publish the annotated frame and transform the marker at image time."""
    (x, y, z), source, _, valid = observation.reading()
    if not valid:
        reject(image, "lost")
        return
    preview = bridge.cv2_to_imgmsg(pixels, "rgb8")
    preview.header = image.header
    annotated.publish(preview)
    transformed(pose(image, source, (x, y, z)))


def pose(
    image: Image, source: str, position: tuple[float, float, float]
) -> PoseStamped:
    """Build the marker pose in the original camera frame."""
    goal = PoseStamped()
    goal.header.stamp = image.header.stamp
    goal.header.frame_id = source
    goal.pose.position = Point(x=position[0], y=position[1], z=position[2])
    goal.pose.orientation.w = 1.0
    return goal


def transformed(goal: PoseStamped) -> None:
    """Transform the observed position and publish it when TF is available."""
    try:
        transform = buffer.lookup_transform(
            frame, goal.header.frame_id, Time.from_msg(goal.header.stamp)
        )
    except ExtrapolationException as error:
        extrapolated(
            error, goal.header.frame_id, Time.from_msg(goal.header.stamp)
        )
    except TransformException as error:
        unavailable(error)
    else:
        publish(do_transform_pose_stamped(goal, transform))


def extrapolated(
    error: ExtrapolationException, source: str, stamp: Time
) -> None:
    """Allow a frame to wait for TF without reporting a transform outage."""
    if not awaiting(source, stamp):
        unavailable(error)


def awaiting(source: str, stamp: Time) -> bool:
    """Check whether the image is newer than the latest available transform."""
    try:
        latest = buffer.lookup_transform(frame, source, Time())
    except TransformException:
        return False
    return Time.from_msg(latest.header.stamp) < stamp


def publish(target: PoseStamped) -> None:
    """Publish the target pose, diagnostic marker and detection state."""
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
synchronizer = ApproximateTimeSynchronizer(
    [images, depths, calibrations], 5, 0.02
)
synchronizer.registerCallback(observe)
status.publish(String(data="waiting"))
node.create_timer(0.1, expire)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    destroy()
    try_shutdown()
