"""Tests for connecting target following to the workshop."""

from pathlib import Path

import yaml
from defusedxml import ElementTree
from hamcrest import assert_that, equal_to


def test_follow_sends_both_command_sources_through_the_mux() -> None:
    """Manual commands take priority over Nav2 before reaching the driver."""
    root = Path(__file__).parents[1]
    launch = ElementTree.parse(root / "launch/follow.launch.xml")
    config = yaml.safe_load((root / "config/follow.yaml").read_text())
    mux = config["twist_mux"]["ros__parameters"]
    assert_that(
        (
            [
                arg.get("value")
                for arg in launch.findall("./include/arg[@name='panel_topic']")
            ],
            [
                remap.get("to")
                for remap in launch.findall(
                    "./node[@name='controller_server']/remap[@from='cmd_vel']"
                )
            ],
            [
                remap.get("to")
                for remap in launch.findall(
                    "./node[@name='twist_mux']/remap[@from='cmd_vel_out']"
                )
            ],
            mux["topics"]["manual"]["priority"]
            > mux["topics"]["navigation"]["priority"],
            mux["use_stamped"],
            config["controller_server"]["ros__parameters"][
                "enable_stamped_cmd_vel"
            ],
        ),
        equal_to(
            (
                ["/cmd_vel_manual"],
                ["/cmd_vel_nav"],
                ["/cmd_vel"],
                True,
                False,
                False,
            )
        ),
        "following did not route manual and navigation commands safely",
    )


def test_follow_drops_previous_goal_updates_when_restarted() -> None:
    """A fresh behavior tree starts with an empty GoalUpdater."""
    assert_that(
        yaml.safe_load(
            (Path(__file__).parents[1] / "config/follow.yaml").read_text()
        )["bt_navigator"]["ros__parameters"].get("always_reload_bt_xml"),
        equal_to(obj=True),
        "following did not reset its behavior tree for a new goal",
    )


def test_follow_forwards_the_cube_option_to_the_workshop() -> None:
    """The follow launch turns on the cube through mapping and workshop."""
    launch = Path(__file__).parents[1] / "launch"
    assert_that(
        [
            ElementTree.parse(launch / f"{name}.launch.xml").find(
                path + "[@name='fake_target'][@value='$(var fake_target)']"
            )
            is not None
            for name, path in (
                ("follow", "./include/arg"),
                ("mapping", "./include/arg"),
                ("workshop", "./node[@exec='main.py']/param"),
                ("workshop", "./node[@exec='panel.py']/param"),
            )
        ],
        equal_to([True, True, True, True]),
        "the follow launch did not enable both ends of cube control",
    )


def test_follow_view_displays_the_cube_marker() -> None:
    """The map displays the simulated cube whenever its marker arrives."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "rviz/follow.rviz").read_text()
    )
    assert_that(
        [
            (display["Enabled"], display["Topic"]["Value"])
            for display in config["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/Marker"
        ],
        equal_to([(True, "/cube/marker")]),
        "RViz did not display the simulated cube marker",
    )


def test_follow_launch_connects_both_costmaps_to_the_selected_scan() -> None:
    """The planner and controller both remap the configured scan topic."""
    root = ElementTree.parse(
        Path(__file__).parents[1] / "launch/follow.launch.xml"
    ).getroot()
    assert_that(
        [
            root.find(f"./node[@name='{name}']/remap[@from='/scan']")
            is not None
            for name in ("planner_server", "controller_server")
        ],
        equal_to([True, True]),
        "the selected scan did not reach both Nav2 costmaps",
    )


def test_follow_costmaps_accept_scan_points_above_the_floor() -> None:
    """Both observation sources allow the D415 scan plane above the floor."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/follow.yaml").read_text()
    )
    assert_that(
        [
            config[name][name]["ros__parameters"]["obstacle_layer"]["scan"].get(
                "max_obstacle_height"
            )
            for name in ("local_costmap", "global_costmap")
        ],
        equal_to([0.2, 0.2]),
        "the costmap sources did not accept above-floor camera observations",
    )


def test_follow_starts_without_a_mapping_process() -> None:
    """Following starts the workshop directly without a SLAM transform."""
    launch = ElementTree.parse(Path(__file__).parents[1] / "launch/follow.launch.xml")
    assert_that(
        launch.find("./include").get("file"),
        equal_to("$(find-pkg-share robocell_sim)/launch/workshop.launch.xml"),
        "following did not remove its mapping dependency",
    )


def test_follow_uses_rolling_windows_in_odometry() -> None:
    """Neither navigation costmap needs a static map."""
    config = yaml.safe_load((Path(__file__).parents[1] / "config/follow.yaml").read_text())
    assert_that(
        [(config[name][name]["ros__parameters"]["global_frame"],
          config[name][name]["ros__parameters"].get("rolling_window"),
          config[name][name]["ros__parameters"]["plugins"])
         for name in ("global_costmap", "local_costmap")],
        equal_to([("odom", True, ["obstacle_layer", "inflation_layer"])] * 2),
        "following did not use rolling costmaps independent of a static map",
    )


def test_follow_uses_mppi_for_differential_drive() -> None:
    """The controller samples trajectories for a differential drive."""
    config = yaml.safe_load((Path(__file__).parents[1] / "config/follow.yaml").read_text())
    assert_that(
        (config["controller_server"]["ros__parameters"]["FollowPath"]["plugin"],
         config["controller_server"]["ros__parameters"]["FollowPath"].get("motion_model")),
        equal_to(("nav2_mppi_controller::MPPIController", "DiffDrive")),
        "following did not use differential-drive MPPI",
    )


def test_follow_view_shows_navigation_costs_and_paths() -> None:
    """The navigation view shows current costs instead of a SLAM map."""
    config = yaml.safe_load((Path(__file__).parents[1] / "rviz/follow.rviz").read_text())
    assert_that(
        (config["Visualization Manager"]["Global Options"]["Fixed Frame"],
         [display["Topic"]["Value"] for display in config["Visualization Manager"]["Displays"]
          if display["Class"] == "rviz_default_plugins/Map"]),
        equal_to(("odom", ["/global_costmap/costmap", "/local_costmap/costmap"])),
        "the navigation view did not display rolling costs in odometry",
    )


def test_follow_displays_a_live_depth_cloud_without_mapping() -> None:
    """A depth projection supplies the RViz cloud without a SLAM process."""
    root = Path(__file__).parents[1]
    launch = ElementTree.parse(root / "launch/follow.launch.xml")
    config = yaml.safe_load((root / "rviz/follow.rviz").read_text())
    assert_that(
        (launch.find("./node[@exec='point_cloud_xyz_node']") is not None,
         [(display["Topic"]["Value"], display["Decay Time"])
          for display in config["Visualization Manager"]["Displays"]
          if display["Class"] == "rviz_default_plugins/PointCloud2"]),
        equal_to((True, [("/camera/depth/points", 0)])),
        "the follow view did not show a live depth cloud without mapping",
    )


def test_follow_view_keeps_the_scene_fixed_while_the_robot_follows() -> None:
    """Object motion remains visible relative to the stationary odometry grid."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "rviz/follow.rviz").read_text()
    )
    assert_that(
        config["Visualization Manager"]["Views"]["Current"]["Target Frame"],
        equal_to("odom"),
        "the follow view did not anchor object motion to the stationary scene",
    )


def test_follow_decimates_the_display_cloud_without_changing_navigation_depth() -> None:
    """The cloud uses a smaller calibrated image while navigation keeps raw depth."""
    launch = ElementTree.parse(
        Path(__file__).parents[1] / "launch/follow.launch.xml"
    )
    assert_that(
        (
            [param.get("value") for param in launch.findall(
                "./node[@exec='crop_decimate_node']/param[@name='decimation_x']"
            )],
            [param.get("value") for param in launch.findall(
                "./node[@exec='crop_decimate_node']/param[@name='decimation_y']"
            )],
            launch.find("./node[@exec='point_cloud_xyz_node']/remap[@from='image_rect']").get("to"),
            launch.find("./node[@exec='depthimage_to_laserscan_node']/remap[@from='depth']").get("to"),
        ),
        equal_to((["4"], ["4"], "/camera/depth/preview/image_raw", "/camera/depth/image_raw")),
        "the cloud did not reduce display traffic independently of navigation",
    )
