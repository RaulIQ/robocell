"""Tests for the launch file of the workshop."""

import math
from pathlib import Path

from defusedxml import ElementTree
from hamcrest import assert_that, close_to, equal_to


def test_workshop_launch_enables_graphical_control_by_default() -> None:
    """The workshop offers graphical control at modest initial speeds."""
    assert_that(
        {
            argument.get("name"): argument.get("default")
            for argument in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("arg")
            if argument.get("name") in {"teleop", "speed", "turn"}
        },
        equal_to({"teleop": "true", "speed": "0.06", "turn": "0.3"}),
        "launch did not enable graphical control at the selected speeds",
    )


def test_workshop_launch_opens_an_optional_control_panel() -> None:
    """Keyboard control opens our panel directly with an explicit off switch."""
    assert_that(
        [
            (
                node.get("pkg"),
                node.get("if"),
                node.get("launch-prefix"),
                {
                    param.get("name"): param.get("value")
                    for param in node.iter("param")
                },
            )
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "panel.py"
        ],
        equal_to(
            [
                (
                    "robocell_sim",
                    "$(var teleop)",
                    None,
                    {"speed": "$(var speed)", "turn": "$(var turn)"},
                )
            ]
        ),
        "launch did not open the panel directly with selected speeds",
    )


def test_workshop_launch_disables_extra_windows_by_default() -> None:
    """The plots remain opt-in alongside the viewer, RViz and panel."""
    assert_that(
        {
            argument.get("name"): argument.get("default")
            for argument in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("arg")
            if argument.get("name") in {"route", "camera", "radar", "survey"}
        },
        equal_to(
            {
                "route": "false",
                "camera": "false",
                "radar": "false",
                "survey": "false",
            }
        ),
        "launch did not disable extra windows by default",
    )


def test_workshop_launch_passes_window_flags_to_the_simulator() -> None:
    """Window flags reach the simulator as arguments rather than parameters."""
    assert_that(
        [
            node.get("args")
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "main.py"
        ],
        equal_to(
            [
                (
                    "--headless=$(var headless) --duration=$(var duration) "
                    "--route=$(var route) --camera=$(var camera) "
                    "--radar=$(var radar) --survey=$(var survey)"
                )
            ]
        ),
        "launch did not pass requested window flags to the simulator",
    )


def test_workshop_launch_fixes_odom_in_the_world() -> None:
    """A static transform hangs odom from the world frame of MuJoCo."""
    assert_that(
        [
            node.get("args", "").split()[-4:]
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "static_transform_publisher"
        ],
        equal_to([["--frame-id", "world", "--child-frame-id", "odom"]]),
        "launch file did not hang odom from the world frame",
    )


def test_workshop_launch_starts_odom_where_the_robot_starts() -> None:
    """Odom sits at the start keyframe of the workshop, x, y and heading."""
    qpos = [
        float(value)
        for value in next(
            key.get("qpos", "")
            for key in ElementTree.parse(
                Path(__file__).parents[1] / "models/waffle_pi/workshop.xml"
            ).iter("key")
            if key.get("name") == "start"
        ).split()
    ]
    args = next(
        node.get("args", "").split()
        for node in ElementTree.parse(
            Path(__file__).parents[1] / "launch/workshop.launch.xml"
        ).iter("node")
        if node.get("exec") == "static_transform_publisher"
    )
    assert_that(
        math.dist(
            [
                float(args[args.index(flag) + 1])
                for flag in ("--x", "--y", "--yaw")
            ],
            (qpos[0], qpos[1], 2 * math.atan2(qpos[6], qpos[3])),
        ),
        close_to(0.0, 1e-6),
        "odom did not sit where the robot starts in the workshop",
    )


def test_workshop_launch_runs_the_odometry_on_simulated_time() -> None:
    """The odometry starts with the simulator and follows simulated time."""
    assert_that(
        [
            {
                param.get("name"): param.get("value")
                for param in node.iter("param")
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "odometry.py"
        ],
        equal_to([{"use_sim_time": "true"}]),
        "launch file did not run the odometry on simulated time",
    )


def test_workshop_launch_runs_the_imu_driver_on_simulated_time() -> None:
    """The IMU driver starts with the simulator and stamps simulated time."""
    assert_that(
        [
            {
                param.get("name"): param.get("value")
                for param in node.iter("param")
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "imu.py"
        ],
        equal_to(
            [
                {
                    "use_sim_time": "true",
                    "interface": "$(var can_interface)",
                    "channel": "$(var can_channel)",
                    "gyro_variance": "$(var gyro_variance)",
                    "accelerometer_variance": "$(var accelerometer_variance)",
                }
            ]
        ),
        "launch file did not run the IMU driver on simulated time "
        "with the variances of its arguments",
    )


def test_workshop_launch_passes_the_noise_to_the_simulator() -> None:
    """The simulator gets the bias, both noises and the seed from arguments."""
    assert_that(
        [
            {
                param.get("name"): param.get("value")
                for param in node.iter("param")
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "main.py"
        ],
        equal_to(
            [
                {
                    "interface": "$(var can_interface)",
                    "channel": "$(var can_channel)",
                    "left_node": "$(var left_node)",
                    "right_node": "$(var right_node)",
                    "lidar_port": "$(var lidar_port)",
                    "gyro_bias": "$(var gyro_bias)",
                    "gyro_noise": "$(var gyro_noise)",
                    "accelerometer_noise": "$(var accelerometer_noise)",
                    "seed": "$(var seed)",
                }
            ]
        ),
        "launch file did not pass the noise arguments to the simulator",
    )


def test_workshop_launch_defaults_the_variances_to_the_noises_squared() -> None:
    """Unless set by hand, the driver reports the variance the board adds."""
    assert_that(
        {
            argument.get("name"): argument.get("default")
            for argument in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("arg")
            if argument.get("name", "").endswith("_variance")
        },
        equal_to(
            {
                "gyro_variance": "$(eval '$(var gyro_noise) ** 2')",
                "accelerometer_variance": (
                    "$(eval '$(var accelerometer_noise) ** 2')"
                ),
            }
        ),
        "variances did not default to the squares of the noises",
    )


def test_workshop_launch_runs_the_filter_with_its_configuration() -> None:
    """The EKF starts with the workshop and loads the installed sensor setup."""
    assert_that(
        [
            (
                node.get("pkg"),
                node.get("name"),
                [param.attrib for param in node.iter("param")],
            )
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "ekf_node"
        ],
        equal_to(
            [
                (
                    "robot_localization",
                    "ekf_filter_node",
                    [
                        {
                            "from": (
                                "$(find-pkg-share robocell_sim)/config/ekf.yaml"
                            )
                        }
                    ],
                )
            ]
        ),
        "launch file did not start the EKF with its sensor configuration",
    )


def test_workshop_launch_shares_the_named_lidar_endpoint() -> None:
    """Simulator and driver receive one explicit endpoint on simulated time."""
    assert_that(
        [
            {
                param.get("name"): param.get("value")
                for param in node.iter("param")
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "lidar.py"
        ],
        equal_to([{"use_sim_time": "true", "port": "$(var lidar_port)"}]),
        "launch did not share the named UART endpoint with the lidar driver",
    )


def test_workshop_launch_starts_one_guarded_canopen_joint_publisher() -> None:
    """The normal launch enables the CAN driver and its collision guard."""
    assert_that(
        [
            {
                param.get("name"): param.get("value")
                for param in node.iter("param")
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "motor_driver.py"
        ],
        equal_to(
            [
                {
                    "use_sim_time": "true",
                    "interface": "$(var can_interface)",
                    "channel": "$(var can_channel)",
                    "left_node": "$(var left_node)",
                    "right_node": "$(var right_node)",
                    "collision_guard": "true",
                }
            ]
        ),
        "workshop did not start one guarded CANopen joint-feedback driver",
    )
