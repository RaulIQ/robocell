"""Install the robocell_sim package for colcon and ros2 run."""

from pathlib import Path

from setuptools import find_packages, setup

setup(
    name="robocell_sim",
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/robocell_sim"],
        ),
        ("share/robocell_sim", ["package.xml"]),
        *[
            (
                str(Path("share/robocell_sim") / folder.relative_to(".")),
                [
                    str(path)
                    for path in sorted(folder.iterdir())
                    if path.is_file()
                ],
            )
            for folder in sorted(
                {
                    path.parent
                    for root in ("config", "launch", "models", "rviz")
                    for path in Path(root).rglob("*")
                    if path.is_file()
                }
            )
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RaulIQ",
    maintainer_email="raulalimbekov@gmail.com",
    description="MuJoCo simulation of a TurtleBot3 Waffle Pi",
    license="Apache-2.0",
    tests_require=["pytest"],
    scripts=[
        "robocell_sim/main.py",
        "robocell_sim/imu.py",
        "robocell_sim/odometry.py",
    ],
)
