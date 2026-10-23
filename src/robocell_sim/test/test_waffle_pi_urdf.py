"""Tests for the URDF of the TurtleBot3 Waffle Pi against its MJCF."""

from pathlib import Path

import mujoco
from defusedxml import ElementTree
from hamcrest import assert_that, empty, equal_to


def test_urdf_names_the_wheel_joints_of_the_mjcf() -> None:
    """Both descriptions turn the wheels on joints of the same names."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models/waffle_pi/waffle_pi.xml")
    )
    assert_that(
        sorted(
            joint.get("name", "")
            for joint in ElementTree.parse(
                Path(__file__).parents[1] / "models/waffle_pi/waffle_pi.urdf"
            ).iter("joint")
            if joint.get("type") == "continuous"
        ),
        equal_to(
            sorted(
                model.joint(index).name
                for index in range(model.njnt)
                if model.jnt_type[index] == mujoco.mjtJoint.mjJNT_HINGE
            )
        ),
        "URDF did not name the wheel joints of the MJCF",
    )


def test_urdf_places_the_lidar_where_the_mjcf_does() -> None:
    """The scan joint of the URDF puts base_scan at the MJCF body."""
    assert_that(
        [
            round(float(value), 9)
            for value in ElementTree.parse(
                Path(__file__).parents[1] / "models/waffle_pi/waffle_pi.urdf"
            )
            .findall("joint[@name='scan_joint']/origin")[0]
            .get("xyz", "")
            .split()
        ],
        equal_to(
            [
                round(float(value), 9)
                for value in mujoco.MjModel.from_xml_path(
                    str(
                        Path(__file__).parents[1]
                        / "models/waffle_pi/waffle_pi.xml"
                    )
                )
                .body("base_scan")
                .pos
            ]
        ),
        "URDF did not place the lidar where the MJCF does",
    )


def test_urdf_places_the_camera_where_the_mjcf_does() -> None:
    """The camera link and RGB frame of the URDF add up to the MJCF camera."""
    tree = ElementTree.parse(
        Path(__file__).parents[1] / "models/waffle_pi/waffle_pi.urdf"
    )
    assert_that(
        [
            round(float(link) + float(frame), 9)
            for link, frame in zip(
                tree.findall("joint[@name='camera_joint']/origin")[0]
                .get("xyz", "")
                .split(),
                tree.findall("joint[@name='camera_rgb_joint']/origin")[0]
                .get("xyz", "")
                .split(),
                strict=True,
            )
        ],
        equal_to(
            [
                round(float(value), 9)
                for value in mujoco.MjModel.from_xml_path(
                    str(
                        Path(__file__).parents[1]
                        / "models/waffle_pi/waffle_pi.xml"
                    )
                )
                .camera("camera_rgb")
                .pos
            ]
        ),
        "URDF did not place the camera where the MJCF does",
    )


def test_urdf_places_the_imu_where_the_mjcf_does() -> None:
    """The IMU joint of the URDF puts imu_link at the MJCF site."""
    assert_that(
        [
            round(float(value), 9)
            for value in ElementTree.parse(
                Path(__file__).parents[1] / "models/waffle_pi/waffle_pi.urdf"
            )
            .findall("joint[@name='imu_joint']/origin")[0]
            .get("xyz", "")
            .split()
        ],
        equal_to(
            [
                round(float(value), 9)
                for value in mujoco.MjModel.from_xml_path(
                    str(
                        Path(__file__).parents[1]
                        / "models/waffle_pi/waffle_pi.xml"
                    )
                )
                .site("imu")
                .pos
            ]
        ),
        "URDF did not place the IMU where the MJCF does",
    )


def test_urdf_finds_every_mesh_in_the_package() -> None:
    """Each package URI of a URDF mesh points at a file of robocell_sim."""
    assert_that(
        [
            uri
            for uri in (
                mesh.get("filename", "")
                for mesh in ElementTree.parse(
                    Path(__file__).parents[1]
                    / "models/waffle_pi/waffle_pi.urdf"
                ).iter("mesh")
            )
            if not (
                Path(__file__).parents[1]
                / uri.removeprefix("package://robocell_sim/")
            ).is_file()
        ],
        empty(),
        "URDF did not find every mesh in the package",
    )
