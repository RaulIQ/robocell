# TurtleBot3 Waffle Pi Description (MJCF)

This package holds a MuJoCo description and a URDF of the
[TurtleBot3 Waffle Pi](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
by [ROBOTIS](https://www.robotis.com/).
It is written by hand from `turtlebot3_waffle_pi.urdf` in
[ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
at commit `fc817ce3073af1d6032397c64504134882af5e9a`,
and it reuses the STL meshes of that package under the Apache 2.0 license in
[LICENSE](./LICENSE).

## URDF to MJCF derivation steps

1. Copied `waffle_pi_base.stl`, `left_tire.stl`, `right_tire.stl` and `lds.stl`
   from `turtlebot3_description/meshes` into `assets` and kept them for visuals only.
2. Kept link names, joint names, poses, masses and inertias from the URDF,
   merging the fixed camera links into `base_link`.
3. Replaced the box casters, which float above the floor in the URDF,
   with frictionless spheres that touch it.
4. Added `velocity` actuators `wheel_left` and `wheel_right`, in radians per second,
   limited to 7.88 rad/s, which is the 0.26 m/s top speed over the 0.033 m wheel radius.
5. Added an estimated `armature` of 0.005 to the wheel joints for the gearbox inertia
   and switched to the `implicitfast` integrator, since without both the light wheels
   and their velocity servos made the simulation unstable.
6. Added the `camera_rgb` camera at the URDF `camera_rgb_frame`, looking forward.
7. Added `jointpos` sensors `wheel_left_encoder` and `wheel_right_encoder` on the wheel joints,
   in radians, to stand in for the encoders of the Dynamixel XM430 wheel servos.
8. Added the `imu_gyro` and `imu_accelerometer` sensors on the `imu` site at the URDF
   `imu_link`, in rad/s and m/s², to stand in for the IMU of the OpenCR board;
   the accelerometer includes gravity, so at rest it reads about 9.81 m/s² up.

## URDF

`waffle_pi.urdf` is `turtlebot3_waffle_pi.urdf` from the same commit,
expanded by `xacro` with an empty `namespace`, with its mesh paths pointed at
`package://robocell_sim/models/waffle_pi/assets/`,
so that MuJoCo and RViz load the same STL files.
`test/test_waffle_pi_urdf.py` checks that both descriptions name the same wheel joints
and place the lidar, the camera and the IMU alike.
