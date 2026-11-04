"""Regressions for the fixed-grid, motion-compensated SLAM scan."""

import copy
import math
from typing import Any

import pytest
from hamcrest import assert_that, close_to, equal_to
from rclpy.time import Time
from robocell_sim.ros.implementations.lidar.rectified import Rectified
from robocell_sim.ros.implementations.lidar.transformed import Transformed
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_ros import TransformException  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.kept import Kept
from test.trajectory import Trajectory


@pytest.mark.parametrize("counts", [(381, 382, 381), (382, 382, 382)])
def test_rectified_preserves_the_grid_across_shifted_turns(
    counts: tuple[int, ...],
) -> None:
    """Neither sample counts nor first angles can change SLAM metadata."""
    messages: list[Any] = []
    for offset, count in enumerate(counts):
        Rectified(
            LaserScan(
                header=Header(
                    stamp=Time(seconds=1).to_msg(), frame_id="base_scan"
                ),
                angle_min=0.013 * offset,
                angle_increment=-math.tau / count,
                time_increment=0.000476,
                scan_time=count * 0.000476,
                range_min=0.15,
                range_max=12.0,
                ranges=[2.73] * count,
            ),
            Transformed(
                Trajectory(
                    1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0
                ).buffer(),
                "base_scan",
                "odom",
            ),
            360,
            Kept(messages),
        ).publish()
    assert_that(
        [
            (
                len(message.ranges),
                message.angle_min,
                message.angle_max,
                message.angle_increment,
            )
            for message in messages
        ],
        equal_to(
            [
                (
                    360,
                    messages[0].angle_min,
                    messages[0].angle_max,
                    messages[0].angle_increment,
                )
            ]
            * 3
        ),
        "successive scans did not retain one fixed angular grid",
    )


def test_rectified_places_clockwise_returns_by_their_angles() -> None:
    """An asymmetric scan places obstacles on their geometric bearings."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_min=math.pi / 2,
            angle_increment=-math.pi / 2,
            time_increment=0.001,
            range_min=0.15,
            range_max=12.0,
            ranges=[1.37, 2.81, 3.49, 4.63],
        ),
        Transformed(
            Trajectory(1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0).buffer(),
            "base_scan",
            "odom",
        ),
        4,
        Kept(messages),
    ).publish()
    assert_that(
        list(messages[0].ranges),
        equal_to(list(LaserScan(ranges=[4.63, 3.49, 2.81, 1.37]).ranges)),
        "clockwise returns did not retain their geometric bearings",
    )


def test_rectified_deskews_a_turn_to_the_last_ray_pose() -> None:
    """Turning acquisition is transformed before angular binning."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_increment=-math.pi / 2,
            time_increment=0.1,
            scan_time=0.4,
            range_min=0.15,
            range_max=12.0,
            ranges=[2.73, math.inf, math.inf, math.inf],
        ),
        Transformed(
            Trajectory(
                1_000_000_000, 1_300_000_004, (0.0, 0.0), math.pi / 2
            ).buffer(),
            "base_scan",
            "odom",
        ),
        4,
        Kept(messages),
    ).publish()
    assert_that(
        list(messages[0].ranges),
        equal_to(
            list(LaserScan(ranges=[math.inf, 2.73, math.inf, math.inf]).ranges)
        ),
        "turning scan did not move the first return into the reference pose",
    )


def test_rectified_accounts_for_translation_during_acquisition() -> None:
    """The common origin changes ranges as well as bearings."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_increment=-math.pi,
            time_increment=0.125,
            range_min=0.15,
            range_max=12.0,
            ranges=[3.75, math.inf],
        ),
        Transformed(
            Trajectory(1_000_000_000, 1_125_000_000, (0.5, 0.0), 0.0).buffer(),
            "base_scan",
            "odom",
        ),
        4,
        Kept(messages),
    ).publish()
    assert_that(
        messages[0].ranges[2],
        close_to(3.25, 1e-6),
        "translation did not change the range from the reference origin",
    )


def test_rectified_uses_nearest_valid_return_in_a_collision() -> None:
    """Invalid returns cannot hide the nearest obstacle in a shared bin."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_increment=-0.001,
            time_increment=0.001,
            range_min=0.15,
            range_max=12.0,
            ranges=[3.75, math.nan, 0.0, math.inf, 1.25, 13.0, -math.inf],
            intensities=[7.0, 8.0, 9.0, 10.0, 17.0, 18.0, 19.0],
        ),
        Transformed(
            Trajectory(1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0).buffer(),
            "base_scan",
            "odom",
        ),
        4,
        Kept(messages),
    ).publish()
    assert_that(
        (list(messages[0].ranges), list(messages[0].intensities)),
        equal_to(([math.inf, math.inf, 1.25, math.inf], [0.0, 0.0, 17.0, 0.0])),
        "collision did not retain the nearest valid return and its intensity",
    )


def test_rectified_stamps_the_snapshot_at_the_last_ray() -> None:
    """Positive bearings describe one snapshot at the reference time."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_increment=-math.pi,
            time_increment=0.125,
            scan_time=0.25,
            range_min=0.15,
            range_max=12.0,
            ranges=[2.75, 3.25],
        ),
        Transformed(
            Trajectory(1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0).buffer(),
            "base_scan",
            "odom",
        ),
        4,
        Kept(messages),
    ).publish()
    assert_that(
        (
            Time.from_msg(messages[0].header.stamp).nanoseconds,
            messages[0].time_increment,
            messages[0].scan_time,
            messages[0].header.frame_id,
        ),
        equal_to((1_125_000_000, 0.0, 0.25, "base_scan")),
        "output did not describe an instantaneous scan at the last acquisition",
    )


def test_rectified_cannot_mutate_the_raw_message() -> None:
    """The raw topic remains an independent acquisition record."""
    scan = LaserScan(
        header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
        angle_increment=-math.pi,
        time_increment=0.125,
        range_min=0.15,
        range_max=12.0,
        ranges=[2.75, 3.25],
        intensities=[13.0, 23.0],
    )
    original = copy.deepcopy(scan)
    Rectified(
        scan,
        Transformed(
            Trajectory(1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0).buffer(),
            "base_scan",
            "odom",
        ),
        4,
        Kept([]),
    ).publish()
    assert_that(
        scan,
        equal_to(original),
        "adapter did not preserve the complete raw message",
    )


@pytest.mark.parametrize(
    ("angle", "slot"), [(0.0, 4), (math.pi / 2, 6), (-math.pi, 0)]
)
def test_rectified_moves_a_return_when_the_first_angle_changes(
    angle: float, slot: int
) -> None:
    """Equal counts cannot hide a shift in actual obstacle bearings."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_min=angle,
            angle_increment=-math.pi,
            time_increment=0.125,
            range_min=0.15,
            range_max=12.0,
            ranges=[2.75, math.inf],
        ),
        Transformed(
            Trajectory(1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0).buffer(),
            "base_scan",
            "odom",
        ),
        8,
        Kept(messages),
    ).publish()
    assert_that(
        messages[0].ranges[slot],
        close_to(2.75, 1e-6),
        "shifted first angle did not move the obstacle to its actual bearing",
    )


def test_rectified_cannot_publish_without_acquisition_tf() -> None:
    """Missing history cannot silently degrade to an uncompensated scan."""
    messages: list[Any] = []
    with pytest.raises(TransformException):
        Rectified(
            LaserScan(
                header=Header(
                    stamp=Time(seconds=1).to_msg(), frame_id="base_scan"
                ),
                angle_increment=-math.pi,
                time_increment=0.125,
                range_min=0.15,
                range_max=12.0,
                ranges=[math.inf, math.inf],
            ),
            Transformed(
                Trajectory(
                    2_000_000_000, 3_000_000_000, (0.0, 0.0), 0.0
                ).buffer(),
                "base_scan",
                "odom",
            ),
            4,
            Kept(messages),
        ).publish()
    assert_that(
        messages,
        equal_to([]),
        "unavailable acquisition TF did not prevent publication",
    )


def test_rectified_leaves_invalid_bins_unoccupied() -> None:
    """No-return samples and empty bins carry infinity with zero strength."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_increment=-math.pi / 2,
            time_increment=0.125,
            range_min=0.15,
            range_max=12.0,
            ranges=[0.0, math.nan, math.inf, 15.0],
        ),
        Transformed(
            Trajectory(1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0).buffer(),
            "base_scan",
            "odom",
        ),
        4,
        Kept(messages),
    ).publish()
    assert_that(
        (list(messages[0].ranges), list(messages[0].intensities)),
        equal_to(([math.inf] * 4, [0.0] * 4)),
        "invalid or missing returns did not remain unoccupied",
    )


@pytest.mark.parametrize("count", [0, 1])
def test_rectified_cannot_accept_a_degenerate_grid(count: int) -> None:
    """The fixed grid requires more than one angular position."""
    messages: list[Any] = []
    with pytest.raises(ValueError, match="invalid geometry"):
        Rectified(
            LaserScan(
                header=Header(
                    stamp=Time(seconds=1).to_msg(), frame_id="base_scan"
                ),
                angle_increment=-math.pi,
                time_increment=0.125,
                range_min=0.15,
                range_max=12.0,
                ranges=[2.75, 3.25],
            ),
            Transformed(
                Trajectory(
                    1_000_000_000, 2_000_000_000, (0.0, 0.0), 0.0
                ).buffer(),
                "base_scan",
                "odom",
            ),
            count,
            Kept(messages),
        ).publish()
    assert_that(
        messages, equal_to([]), "degenerate grid did not prevent publication"
    )


def test_rectified_uses_each_rays_interpolated_pose() -> None:
    """A middle ray uses its own acquisition pose within the turn."""
    messages: list[Any] = []
    Rectified(
        LaserScan(
            header=Header(stamp=Time(seconds=1).to_msg(), frame_id="base_scan"),
            angle_min=math.pi,
            angle_increment=-math.pi / 2,
            time_increment=0.125,
            range_min=0.15,
            range_max=12.0,
            ranges=[math.inf, 2.75, math.inf],
        ),
        Transformed(
            Trajectory(
                1_000_000_000, 1_250_000_000, (0.0, 0.0), math.pi / 2
            ).buffer(),
            "base_scan",
            "odom",
        ),
        8,
        Kept(messages),
    ).publish()
    assert_that(
        messages[0].ranges[5],
        close_to(2.75, 1e-6),
        "middle ray did not use the interpolated acquisition pose",
    )
