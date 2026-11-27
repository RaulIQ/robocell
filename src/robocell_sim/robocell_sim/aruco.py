#!/usr/bin/env python3
"""Publish fresh ArUco target observations in the navigation frame."""

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped  # type: ignore[import-untyped]  # ROS generates message modules without types
from message_filters import ApproximateTimeSynchronizer, Subscriber  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_geometry_msgs import do_transform_pose_stamped  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from tf2_ros import Buffer, TransformException, TransformListener  # type: ignore[import-untyped]  # Jazzy ships this module without typing information

from robocell_sim.ros.implementations.pose.observed import Observed
from robocell_sim.ros.implementations.settings.declared import Declared

rclpy.init()
node = rclpy.create_node("aruco")
settings = Declared(node)
identifier = settings.whole("marker_id", 0)
if not 0 <= identifier < 50:
    raise ValueError("ArUco marker ID must be between 0 and 49")
frame = settings.text("target_frame", "odom")
observed = Observed(identifier, cv2.aruco.DICT_4X4_50, CvBridge())
output = node.create_publisher(PoseStamped, "/target/pose", 1)
buffer = Buffer(node=node)
listener = TransformListener(buffer, node)


def observe(image: Image, depth: Image, calibration: CameraInfo) -> None:
    """Publish only fresh observations with a transform at the image time."""
    stamp = Time.from_msg(image.header.stamp)
    if not (0 <= (node.get_clock().now() - stamp).nanoseconds <= 500_000_000):
        return
    try:
        observation = observed.observe(image, depth, calibration)
        if not observation.valid():
            return
        position = observation.position()
        goal = PoseStamped()
        goal.header.stamp = image.header.stamp
        goal.header.frame_id = observation.frame()
        goal.pose.position = Point(x=position[0], y=position[1], z=position[2])
        goal.pose.orientation.w = 1.0
        transform = buffer.lookup_transform(frame, observation.frame(), stamp)
        output.publish(do_transform_pose_stamped(goal, transform))
    except (TransformException, ValueError):
        return


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
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
