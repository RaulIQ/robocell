#!/usr/bin/env python3
"""Publish fresh ArUco target observations in the navigation frame."""

from collections.abc import Callable

import cv2
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.utilities import try_shutdown

from robocell_sim.robot.implementations.detection.detection import Detected
from robocell_sim.robot.implementations.detection.projection import Projected
from robocell_sim.robot.implementations.detection.recent import Recent
from robocell_sim.ros.implementations.camera.surveyed import Surveyed
from robocell_sim.ros.implementations.clock.timed import Timed
from robocell_sim.ros.implementations.pose.located import Located
from robocell_sim.ros.implementations.pose.observed import Observed
from robocell_sim.ros.implementations.pose.sighted import (
    Marker,
    Sighted,
    String,
)
from robocell_sim.ros.implementations.settings.declared import Declared
from robocell_sim.ros.messages import (
    Buffer,
    CameraInfo,
    Image,
    PoseStamped,
    TransformListener,
)

_MARKERS = 50

rclpy.init()
node = rclpy.create_node("aruco")
recent = Recent(Timed(node))
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
target = Sighted(
    Timed(node),
    (status, points, output),
    (identifier, info, warning),
    [("waiting", 0)],
)
survey = Surveyed(
    recent,
    observed,
    (bridge, annotated),
    (Located(buffer, frame, target), target.report),
)

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
synchronizer.registerCallback(survey)
status.publish(String(data="waiting"))
node.create_timer(0.1, target.expire)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    destroy()
    try_shutdown()
