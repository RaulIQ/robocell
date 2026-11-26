"""Tests for the workshop mapping launch."""

from pathlib import Path

from defusedxml import ElementTree
from hamcrest import assert_that, equal_to


def test_mapping_launch_removes_the_static_parent_of_odom() -> None:
    """SLAM owns map to odom without the workshop world to odom link."""
    assert_that(
        [
            argument.get("value")
            for include in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("include")
            if include.get("file", "").endswith("workshop.launch.xml")
            for argument in include.iter("arg")
            if argument.get("name") == "fixed_odom"
        ],
        equal_to(["false"]),
        "mapping did not remove the static parent of odom",
    )


def test_mapping_launch_activates_slam_on_simulated_time() -> None:
    """The upstream launch configures and activates the mapping lifecycle."""
    assert_that(
        [
            {
                argument.get("name"): argument.get("value")
                for argument in include.iter("arg")
            }
            for include in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("include")
            if include.get("file", "").endswith("online_async_launch.py")
        ],
        equal_to(
            [
                {
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                    "use_sim_time": "true",
                    "slam_params_file": ("$(var slam_params_file)"),
                }
            ]
        ),
        "mapping did not activate SLAM with its simulated-time configuration",
    )


def test_mapping_launch_selects_the_map_view() -> None:
    """RViz uses map as its fixed frame in mapping mode."""
    launch = ElementTree.parse(
        Path(__file__).parents[1] / "launch/mapping.launch.xml"
    )
    assert_that(
        (
            [
                argument.get("default")
                for argument in launch.findall("./arg[@name='rviz_config']")
            ],
            [
                argument.get("value")
                for argument in launch.findall(
                    "./include/arg[@name='rviz_config']"
                )
            ],
        ),
        equal_to(
            (
                ["$(find-pkg-share robocell_sim)/rviz/mapping.rviz"],
                ["$(var rviz_config)"],
            )
        ),
        "mapping did not select and forward the default map view",
    )


def test_mapping_launch_rectifies_driver_scans_on_simulated_time() -> None:
    """Mapping starts the adapter alongside its original driver."""
    assert_that(
        [
            {
                parameter.get("name"): parameter.get("value")
                for parameter in node.iter("param")
                if parameter.get("name") == "use_sim_time"
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("node")
            if node.get("pkg") == "robocell_scan"
            and node.get("exec") == "slam_scan"
        ],
        equal_to([{"use_sim_time": "true"}]),
        "mapping did not start the fixed-grid adapter on simulated time",
    )


def test_mapping_launch_accepts_a_recorded_slam_configuration() -> None:
    """A run can retain its exact SLAM settings without editing the package."""
    assert_that(
        [
            argument.get("value")
            for include in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("include")
            if include.get("file", "").endswith("online_async_launch.py")
            for argument in include.iter("arg")
            if argument.get("name") == "slam_params_file"
        ],
        equal_to(["$(var slam_params_file)"]),
        "mapping did not accept the recorded SLAM configuration",
    )


def test_mapping_launch_selects_the_camera_producer_for_slam() -> None:
    """The camera converter sends its sector scan to the SLAM adapter."""
    root = ElementTree.parse(
        Path(__file__).parents[1] / "launch/mapping.launch.xml"
    ).getroot()
    assert_that(
        [
            node.get("to")
            for node in root.findall(
                "./node[@pkg='depthimage_to_laserscan']/remap[@from='scan']"
            )
        ],
        equal_to(["/scan/depth"]),
        "the depth camera converter did not publish a sector scan",
    )


def test_mapping_launch_starts_slam_toolbox_only_for_lidar() -> None:
    """RTAB-Map replaces SLAM Toolbox when the depth source is selected."""
    assert_that(
        [
            include.get("if")
            for include in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("include")
            if include.get("file", "").endswith("online_async_launch.py")
        ],
        equal_to(['$(eval "\'$(var scan_source)\' == \'lidar\'")']),
        "depth mode did not leave map to odom to RTAB-Map alone",
    )


def test_mapping_launch_registers_and_synchronizes_rgbd_frames() -> None:
    """Depth is reprojected into the RGB camera before exact synchronization."""
    launch = ElementTree.parse(
        Path(__file__).parents[1] / "launch/mapping.launch.xml"
    )
    nodes = [
        node
        for node in launch.iter("node")
        if node.get("pkg") in {"depth_image_proc", "rtabmap_sync"}
    ]
    assert_that(
        [
            (
                node.get("exec"),
                {
                    parameter.get("name"): parameter.get("value")
                    for parameter in node.iter("param")
                    if parameter.get("name") in {"approx_sync", "decimation"}
                },
                {remap.get("from"): remap.get("to") for remap in node.iter("remap")},
            )
            for node in nodes
        ],
        equal_to(
            [
                (
                    "register_node",
                    {},
                    {
                        "depth/image_rect": "/camera/depth/image_raw",
                        "depth/camera_info": "/camera/depth/camera_info",
                        "rgb/camera_info": "/camera/camera_info",
                        "depth_registered/image_rect": (
                            "/camera/depth_registered/image_raw"
                        ),
                    },
                ),
                (
                    "rgbd_sync",
                    {"approx_sync": "false", "decimation": "4"},
                    {
                        "rgb/image": "/camera/image_raw",
                        "depth/image": "/camera/depth_registered/image_raw",
                        "rgb/camera_info": "/camera/camera_info",
                        "rgbd_image": "/camera/rgbd_image",
                    },
                ),
            ]
        ),
        "RGB and depth were not registered and synchronized into one RGB-D frame",
    )


def test_mapping_launch_uses_visual_pose_without_duplicate_odom_tf() -> None:
    """RGB-D odometry feeds EKF while RTAB-Map alone owns map to odom."""
    launch = ElementTree.parse(
        Path(__file__).parents[1] / "launch/mapping.launch.xml"
    )
    nodes = [
        node
        for node in launch.iter("node")
        if node.get("exec") in {"rgbd_odometry", "rtabmap"}
    ]
    assert_that(
        [
            (
                node.get("exec"),
                {
                    parameter.get("name"): parameter.get("value")
                    for parameter in node.iter("param")
                    if parameter.get("name")
                    in {
                        "frame_id",
                        "odom_frame_id",
                        "map_frame_id",
                        "publish_tf",
                        "publish_null_when_lost",
                        "subscribe_rgbd",
                    }
                },
                {
                    remap.get("from"): remap.get("to")
                    for remap in node.iter("remap")
                    if remap.get("from") in {"odom", "rgbd_image"}
                },
            )
            for node in nodes
        ],
        equal_to(
            [
                (
                    "rgbd_odometry",
                    {
                        "frame_id": "base_footprint",
                        "odom_frame_id": "odom",
                        "publish_tf": "false",
                        "publish_null_when_lost": "false",
                        "subscribe_rgbd": "true",
                    },
                    {
                        "odom": "/visual/odometry",
                        "rgbd_image": "/camera/rgbd_image",
                    },
                ),
                (
                    "rtabmap",
                    {
                        "frame_id": "base_footprint",
                        "odom_frame_id": "odom",
                        "map_frame_id": "map",
                        "publish_tf": "true",
                        "subscribe_rgbd": "true",
                    },
                    {
                        "odom": "/odometry/filtered",
                        "rgbd_image": "/camera/rgbd_image",
                    },
                ),
            ]
        ),
        "visual odometry and RTAB-Map did not get separate EKF and TF roles",
    )
