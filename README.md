# robocell

## What v0 does

v0 drives a simulated TurtleBot3 Waffle Pi through a 6 by 6 m workshop in
MuJoCo from the keyboard. Its odometry takes the distance from the wheel
encoders and the turn from an integrated gyro, a 360-ray lidar ranges the
room five times a second, a camera renders what the robot sees, and a guard
stops the robot short of anything the lidar sees. Optional windows plot the
route against the ground truth, the latest lidar scan, the camera image and
two maps of the room, by the true pose and by odometry; the viewer marks
every lidar echo, and the console warns when the wheels slip.

![v0 driving a loop through the workshop](docs/v0.gif)

A scripted loop at three times the real speed: the workshop with the lidar
echoes, the robot camera, the lidar scan and the route by odometry against
the MuJoCo ground truth.

## What v0.5 does

v0.5 puts the simulator on ROS 2, so the rest of a robot stack can talk to it
the way it will talk to the real Waffle Pi. One node publishes the simulated
clock, the true pose, the wheel joints, the footprint in TF, the lidar scan
and the camera picture with its calibration, each at its own rate in simulated
time, and takes velocity commands on `/cmd_vel`. One launch file adds the URDF,
a fixed `world` frame and an RViz view with the robot model, its frames, the
true pose, the lidar echoes and the camera picture with the scene drawn over
it. Every stand-in for real hardware is listed under
[Temporary bridges](#temporary-bridges) with what replaces it.

![v0.5 in RViz next to the MuJoCo viewer](docs/v0.5.gif)

RViz on the left and the MuJoCo viewer on the right, both following one drive
through the workshop: the camera picture and the same picture with the scene
drawn over it, the lidar echoes in green and the true pose as a red arrow.

## Layout

The `robocell_sim` package splits into four worlds. Each world keeps all its
interfaces in one `interfaces` folder and sorts its `implementations` into one
folder per device or subject:

| Package    | Role                                       | Implementations                                        |
| ---------- | ------------------------------------------ | ------------------------------------------------------ |
| `hardware` | stands in for the real TurtleBot3          | `physics`, `imu`, `can`, `wheels`, `lidar`, `camera`   |
| `robot`    | would run on a real TurtleBot3 under ROS 2 | `drive`, `command`, `odometry`, `lidar`                |
| `display`  | stands in for RViz and a web console       | `route`, `lidar`, `maps`, `camera`                     |
| `ros`      | carries the simulation to ROS 2            | `wiring`, `clock`, `pose`, `wheels`, `lidar`, `camera` |

For example, the `Imu` interface lives in `hardware/interfaces`, while
`hardware/implementations/imu` holds `Sensed`, which reads it from MuJoCo, and
`Emulated`, the board that sends it on CAN.

Nothing under `robot` imports MuJoCo, and a test keeps it that way. Only `ros`
imports ROS message types, so the other worlds test without them; ROS
generates its message modules without types, so `ros` alone suppresses those
type checks, line by line for mypy and by folder for Pylance.
`main.py` wires objects from all four worlds together.

## Work with Codex

Rebuild and reopen the workspace with **Dev Containers: Rebuild and Reopen in
Container** in VS Code. The container installs the Codex extension and CLI;
`~/.codex` is mounted at `/home/ubuntu/.codex` to keep your configuration,
sign-in and global `AGENTS.md` available across rebuilds.

Open the Codex sidebar to chat, or run `codex` in the container terminal.
If sign-in is required, use the extension or `codex login --device-auth`.
Review and trust the project hooks with `/hooks` in Codex CLI, once for the
host checkout and once for `/ws` inside the container. The shared EO gate
runs after patches and before Codex finishes a turn; on the host it runs
through Docker Compose, and in the container it uses the installed Python.

## Run the simulation

The simulator sends the readings of its IMU board on the virtual CAN bus
`vcan0`, and stops at start if that bus is missing. Install the systemd unit
from `docker/vcan0.service` on the host once, from the workspace root, and it
brings `vcan0` up on every boot:

```console
sudo cp docker/vcan0.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vcan0.service
```

The unit loads the `vcan` module, adds `vcan0` unless it is already there and
sets it up; `sudo systemctl stop vcan0` deletes the bus again. Without the
unit, the same three steps bring it up by hand until the next boot:

```console
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

The dev container shares the network of the host through `network_mode: host`
in `compose.yaml`, so it sees `vcan0` as soon as the host brings it up.

`robocell_sim` is an `ament_python` package. Build it with colcon from the
workspace root, then start the TurtleBot3 Waffle Pi in the MuJoCo viewer:

```console
cd /ws
python -m colcon build --symlink-install
source install/setup.bash
ros2 run robocell_sim main.py
```

Only the MuJoCo viewer opens by default. Request any of the four extra
windows with `--route`, `--camera`, `--radar` or `--survey`, independently
or together:

```console
ros2 run robocell_sim main.py --route --survey
ros2 run robocell_sim main.py --route --camera --radar --survey
```

Disabled windows create no processes or queues and receive no data. The
ROS camera image, lidar scan and other topics still work without the extra
windows. Use `ros2 run robocell_sim main.py --help` to list the flags; they
can be combined with the usual `--ros-args` options.

Run colcon through `python -m` so it uses the virtual environment that holds
MuJoCo; the plain `colcon` command runs the system Python, which does not.
With `--symlink-install`, edits to the code and the models take effect without
a rebuild; only changes to `setup.py`, `setup.cfg` or `package.xml` need one.

Run the tests the same way, or with the strict style gate from the root:

```console
python -m colcon test --packages-select robocell_sim
python -m colcon test-result --verbose
python -m pytest --eo
```

The robot drives in a closed 6 by 6 m workshop centered at the origin, with
four cabinets, three columns, a charging dock on the west wall and a low
pallet that stays under the lidar scan plane. It starts beside the dock,
facing north.

Steer the robot with the keyboard while the viewer window has focus.
Every press changes the speed by one step, up to the robot limits.

| Key            | Action                              |
| -------------- | ----------------------------------- |
| Up / Down      | forward speed one step up or down   |
| Left / Right   | turn one step to the left or right  |
| Enter          | stop                                |
| Space          | pause or resume the simulation      |
| 9              | show or hide the lidar lines        |

Every half second of simulated time the console logs both wheel encoders
in ticks, at 4096 ticks per wheel revolution, as the Dynamixel XM430 counts them.

With `--route`, a window draws the route of the robot as it drives: the
dashed line is where the wheel encoders and the gyroscope put it, the solid
line is where MuJoCo actually has it, and the title tells how far apart they
are in millimeters. The distance comes from the encoders and the turn from
the gyroscope, whose yaw rate adds up on every physics step.

Every tenth of a second of simulated time the robot also compares how fast
the encoders and the gyroscope say it turns. When the two rates differ by
more than 0.3 rad/s, the wheels slip: the console logs a warning, the route
window marks the spot on the dashed line with a red cross, and the console
logs again once the wheels grip.

With `--camera`, a window shows what the robot camera sees, 640 by 480 pixels
thirty times a second. MuJoCo renders it in a process of its own from the joint
positions the simulation sends, so a slow window drops frames instead of
slowing the robot down. At start the console logs the camera matrix of this
ideal pinhole camera, computed from its vertical field of view, with no
lens distortion.

With `--radar`, a window draws the lidar scan five times a second on a polar
chart, with ahead at the top. The lidar casts 360 level rays from the scanner, one
per degree counterclockwise from ahead as in ROS, and keeps ranges between
0.12 and 3.5 m; the low pallet stays under its scan plane, so only the camera
sees it.

The MuJoCo viewer marks every lidar echo with a yellow dot, repainted with
each scan, and the 9 key adds red lines from the scanner to the dots.

The robot refuses to drive into what the lidar sees. While any echo lies in a
corridor 0.3 m ahead of the scanner and 0.2 m to each side, the forward speed
drops to zero, and an echo in the same corridor behind blocks reversing.
Turning in place stays free, so you can steer away. The low pallet stays
under the scan plane, so the robot can still bump into it.

With `--survey`, a window piles the lidar scans up into two maps of the
workshop, once a second: on the left by the true pose from MuJoCo, on the
right by the pose the
odometry reckons. While the odometry holds, both show the same plan; when the
robot bumps into the pallet and the wheels spin in place, the right map smears
the walls along the way the encoders think the robot went.

The simulator also plays the IMU board of the robot. A hundred times a second
of simulated time, the board reads the gyro and the accelerometer on the `imu`
site, which sits where the URDF puts `imu_link`, and sends one measurement on
`vcan0` as two classic CAN frames: `0x101` with the angular velocity and
`0x102` with the linear acceleration, gravity included. Both frames carry the
same measurement number, which counts up from the start and wraps after 255.
[docs/protocols/imu.md](docs/protocols/imu.md) describes the frames byte by
byte; `candump` from `can-utils` shows them as they go out:

```console
candump -td vcan0
```

Like a cheap MEMS chip, the board does not read its sensors exactly. Its gyro
carries a bias on each axis, drawn once at start from a normal distribution
with a deviation of 0.0075 rad/s and logged in one line, and both sensors add a
fresh normal noise to every axis of every measurement: 0.001 rad/s for the gyro
and 0.02 m/s² for the accelerometer. Each measurement draws its noise exactly
once, so the frames do not depend on how the simulation steps. The `sim` node
takes these as the parameters `gyro_bias`, `gyro_noise` and
`accelerometer_noise`, all standard deviations; the v0 windows still read the
clean MuJoCo gyro.

The bias and the noise come from one generator, seeded by the `seed` parameter.
A negative seed, the default, picks a fresh random one, so every run gets a bias
of its own; the start log names the seed next to the bias, so any run can be
repeated. A fixed seed gives the same bias and the same noise on every run,
which is what an experiment needs to compare two estimators on one gyro. About
one run in five draws a z bias under 0.002 rad/s, too small to see the heading
drift, so pick a seed whose logged bias is large enough.

At rest the accelerometer frame ends in about `52 26 xx 00`, 9.81 m/s² up, and
its z bytes jitter by about twenty units; the z bytes of the gyro frame stay
near the logged bias, in thousandths of rad/s, and jitter by about one unit.
Turning left in place makes the z bytes of the gyro frame, bytes 4 and 5, a
positive number that grows with the turn rate.

The IMU driver reads those frames back, the way a driver on the real robot
would, and publishes them on `/imu` as `sensor_msgs/Imu` in `imu_link`. It
is a node of its own named `imu`, which the launch file starts next to the
simulator, and it knows the board only through the protocol document. Every
5 ms it takes all frames waiting on `vcan0`, pairs the gyro and the
accelerometer frame of each measurement by their number, and publishes the
pair stamped with the time it arrived. The message carries no orientation,
so its orientation covariance starts with -1, and the variances of the gyro
and the accelerometer sit on their diagonals. When frames go missing, the
driver logs one warning per gap with the number of measurements it lost.

```console
ros2 run robocell_sim imu.py --ros-args -p use_sim_time:=true
ros2 topic echo --once /imu
```

Its parameters are `channel` (`vcan0`), `period` (0.005 s), `frame_id`
(`imu_link`), `gyro_variance` (1e-6 (rad/s)²) and `accelerometer_variance`
(4e-4 (m/s²)²), the last two the squares of the default noises of the board.

The simulator publishes ROS 2 topics from `sim` on simulated time:
`/clock` at 100 Hz, the camera at 29.4 Hz and truth and wheel joints at
50 Hz. The separate `lidar` node publishes full UART scans at about 5.5 Hz. The `/clock` topic carries the simulated time as
`rosgraph_msgs/Clock`, so nodes with `use_sim_time` follow the simulation and
stop while it pauses.
The `/ground_truth/odom` topic carries where MuJoCo truly has `base_link` as
`nav_msgs/Odometry`: the pose in the `world` frame and the twist in the frame
of the robot, stamped with the simulated time.

```console
ros2 topic echo --once /clock
ros2 topic echo --once /ground_truth/odom
```

The `/joint_states` topic carries the wheel angles as `sensor_msgs/JointState`.
It counts them from whole encoder ticks, the way the motor driver will report
them over CAN, rather than from the exact MuJoCo joints.

The odometry is a node of its own named `odometry`, which the launch file
starts next to the simulator. It knows the robot only through ROS: it turns
the wheel angles of `/joint_states` back into encoder ticks and both moves and
turns by the wheels alone, leaving the gyro of `/imu` to the EKF, so the filter
counts each sensor once. For every
`/joint_states` it publishes what the wheels reckon, as `nav_msgs/Odometry` on
`/wheel/odometry` with the forward speed and the yaw rate over the last two
readings and their variances, stamped with the time of the wheel angles.
The wheel node publishes no TF; the EKF owns `odom → base_footprint`.
The footprint lies on the floor under `base_link` and keeps only its
heading, so `robot_state_publisher` alone lifts `base_link` 0.01 m above it, as
the URDF says. The odometry starts at zero, so `odom` lies where the robot
starts, and from there it drifts from the true pose as the wheels slip.

```console
ros2 topic echo --once /joint_states
ros2 topic echo --once /wheel/odometry
ros2 run tf2_ros tf2_echo odom base_scan
```

Its parameters are `resolution` (4096 ticks per wheel turn), `wheel_radius`
(0.033 m), `wheel_separation` (0.288 m), `left_joint` and `right_joint`
(`wheel_left_joint` and `wheel_right_joint`), `odom_frame` (`odom`),
`base_frame` (`base_footprint`), and `linear_variance` (7.1e-6 (m/s)²) and
`angular_variance` (7.9e-5 (rad/s)²), the spread it reports for the forward speed
and the yaw rate. These are measured on collision-free simulated routes;
see the [covariance calibration](docs/accuracy/README.md#4-wheel-covariances-and-ekf).
They do not compensate for gyro bias or contact-induced wheel slip.

The `ekf_filter_node` from `robot_localization` fuses the forward speed and
yaw rate of `/wheel/odometry` with the gyro yaw rate of `/imu`. It runs at
50 Hz on simulated time in planar mode, publishes `nav_msgs/Odometry` on
`/odometry/filtered`, and publishes `odom → base_footprint` on `/tf`. Its
configuration lives in `src/robocell_sim/config/ekf.yaml`. It uses neither
the pose derived from the same wheels nor the IMU acceleration or orientation.
The IMU reports no orientation, and its acceleration still includes gravity.
The filter weights both yaw rates by their reported variances; it does not
estimate the gyro bias or correct drift against an external position source.
The v0 route and map windows keep their original encoder and clean-gyro estimate.

```console
ros2 topic echo --once /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

The `lidar` driver reads the emulated RPLIDAR A1 standard-SCAN stream over
UART and alone publishes `/scan` as `sensor_msgs/LaserScan` in `base_scan`.
The A1 profile ranges 0.15–12 m at 5.5 rotations/s, normally 381 or 382
points/rotation. The driver discards the initial partial rotation, bins
each full turn onto a uniform grid, and reports `inf` for invalid returns.
Samples keep their acquisition order: clockwise rotation becomes a negative
ROS angular increment, with positive time between samples. The first-ray
stamp is estimated from ROS reception time and the queried sample period;
it is not a timestamp transmitted by the A1. The MuJoCo scan and map windows
retain their separate 360-ray, 5 Hz, 0.12–3.5 m display/guard path.

```console
ros2 topic echo --once /scan
```

The `/camera/image_raw` topic carries what `camera_rgb` sees as an `rgb8`
`sensor_msgs/Image` of 640 by 480 pixels, sent best effort like any video, and
`/camera/camera_info` carries its ideal pinhole as `sensor_msgs/CameraInfo`,
with no distortion. Both lie in `camera_rgb_optical_frame`. The step count
rounds 1/30 s to 17 physics steps, so the camera runs at 29.4 Hz rather than 30.

```console
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
```

## Watch the robot in RViz

One launch file starts the simulator, `robot_state_publisher` with the URDF from
`models/waffle_pi`, the IMU and lidar drivers, the wheel odometry, the EKF, a static transform that places
`odom` in the MuJoCo `world` where the robot starts, and RViz with a saved
view, all on simulated time:

```console
ros2 launch robocell_sim workshop.launch.xml
```

The launch opens the MuJoCo viewer and RViz by default. Enable extra windows
with the matching launch arguments, all `false` unless requested:

```console
ros2 launch robocell_sim workshop.launch.xml route:=true survey:=true
```

RViz shows the Waffle Pi in the `world` frame with its TF axes, and its wheels
turn along with the wheels in MuJoCo. A red arrow marks the true pose from
`/ground_truth/odom`, green squares mark the lidar echoes from `/scan`, one
panel shows the picture from `/camera/image_raw`, and another draws the lidar,
the frames and the robot over that picture at half opacity, so the echoes land
on the walls the camera sees. Steer it with the arrow keys in the viewer, or
from a second terminal with `teleop_twist_keyboard` on `/cmd_vel`.

The launch file sets the errors of the IMU board and the variances the driver
reports from one place. Its arguments `gyro_bias` (0.0075 rad/s), `gyro_noise`
(0.001 rad/s), `accelerometer_noise` (0.02 m/s²) and `seed` (-1) go to the
simulator, and `gyro_variance` and `accelerometer_variance` go to the driver,
defaulting to the squares of the matching noises. So by default `/imu` reports
exactly the noise the board adds, and one argument changes both:

```console
ros2 launch robocell_sim workshop.launch.xml gyro_noise:=0.005
```

Now the gyro bytes jitter by about five units, and `/imu` reports a gyro
variance of 2.5e-5. Setting a variance by hand breaks that match on purpose,
to study an estimator that trusts the sensor more than it deserves:

```console
ros2 launch robocell_sim workshop.launch.xml gyro_noise:=0.005 gyro_variance:=1e-8
```

Write every value with a decimal point, such as `gyro_bias:=0.0` rather than
`gyro_bias:=0`, since ROS 2 reads a bare `0` as an integer and refuses it for
a parameter declared as a float.

## ROS 2 reference

### Topics

Rates count in simulated time. QoS is what `ros2 topic info -v` reports for
the publisher, or for the subscription on `/cmd_vel`.

| Topic                 | Type                     | Rate          | QoS                              | Why this QoS                                                                                                   |
| --------------------- | ------------------------ | ------------- | -------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `/clock`              | `rosgraph_msgs/Clock`    | 100 Hz        | reliable, keep last 10           | Timers of every `use_sim_time` node step on it; the message is tiny, so delivery costs nothing.                |
| `/ground_truth/odom`  | `nav_msgs/Odometry`      | 50 Hz         | reliable, keep last 10           | It is the reference that estimates are judged against, so a gap would skew the error.                         |
| `/tf`                 | `tf2_msgs/TFMessage`     | 50 Hz, ≤ 20 Hz | reliable, keep last 10 and 100   | TF buffers interpolate between transforms, and a missing one breaks lookups; the footprint comes from the EKF at 50 Hz, the wheels from `robot_state_publisher` at its own 20 Hz cap. |
| `/wheel/odometry`     | `nav_msgs/Odometry`      | 50 Hz         | reliable, keep last 10           | Each pose builds on the one before, and estimators that fuse it expect every one; it is small.                |
| `/odometry/filtered` | `nav_msgs/Odometry` | 50 Hz | reliable, keep last 10 | The EKF pose and velocity after fusing wheel and gyro measurements. |
| `/tf_static`          | `tf2_msgs/TFMessage`     | once          | reliable, transient local        | Fixed frames go out once, so a node that starts late must still get them.                                     |
| `/robot_description`  | `std_msgs/String`        | once          | reliable, transient local        | RViz may start after `robot_state_publisher` and still needs the URDF.                                        |
| `/joint_states`       | `sensor_msgs/JointState` | 50 Hz         | reliable, keep last 10           | A small message; reliable publishing also serves `robot_state_publisher`, which subscribes best effort.        |
| `/scan`               | `sensor_msgs/LaserScan`  | 5.5 Hz          | reliable, keep last 10           | A full turn at 5.5 Hz is small, and reliable publishing serves both RViz and best effort readers like Nav2.   |
| `/camera/image_raw`   | `sensor_msgs/Image`      | 29.4 Hz       | best effort, keep last 5         | A frame is 900 KB and only the newest one matters, so a lost frame is skipped rather than resent late.         |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 29.4 Hz       | reliable, keep last 10           | Readers pair it with each picture; it is small, and reliable serves readers of either kind.                    |
| `/cmd_vel`            | `geometry_msgs/Twist`    | as sent       | reliable, keep last 10           | A lost command would leave the robot on its last speed, so every one must arrive.                              |
| `/imu`                | `sensor_msgs/Imu`        | 100 Hz        | best effort, keep last 5         | Readings come every 10 ms, so a late resend is worth less than the next fresh reading.                         |

### Frames

![TF tree of the simulator](docs/tf_tree.svg)

The tree above comes from `ros2 run tf2_tools view_frames` against the launch
file; the same tree is in [docs/tf_tree.pdf](docs/tf_tree.pdf).

| Frame                      | Meaning                                                                                                                       |
| -------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `world`                    | The MuJoCo world, fixed forever; the true pose lives here, so truth and every estimate can be compared in it.                  |
| `odom`                     | Where odometry counts from; it moves smoothly and never jumps, but drifts from `world`. A static transform pins it to `world` where the robot starts. |
| `base_footprint`           | The robot on the floor: its position and heading, with no height, roll or pitch.                                             |
| `base_link`                | The body of the robot, 0.01 m above the footprint, centered between the wheels.                                              |
| `wheel_left_link`, `wheel_right_link` | The wheels, turned by the angles in `/joint_states`.                                                              |
| `caster_back_left_link`, `caster_back_right_link` | The rear casters, fixed to the body.                                                                  |
| `base_scan`                | The lidar; positive ROS angles are counterclockwise from x, while the scan array follows clockwise acquisition.                                                          |
| `imu_link`                 | The IMU on the control board, where the IMU driver publishes `/imu`.                                                         |
| `camera_link`, `camera_rgb_frame` | The camera body and its lens, x forward as on the robot.                                                              |
| `camera_rgb_optical_frame` | The lens in optical axes, z forward, x right and y down, where pictures and their calibration live.                          |

### Temporary bridges

Some topics come straight out of MuJoCo where the real robot will have a driver
or an estimator. Each bridge stays until its successor lands:

| Bridge today                                    | Replaced by                               |
| ----------------------------------------------- | ----------------------------------------- |
| `/cmd_vel` straight into `data.ctrl`            | `motor_driver` over CANopen (week 4)      |
| `/joint_states` from the simulator              | `motor_driver` (week 4)                   |
| the camera straight from `Rendered`             | stays, as a simplification of the project |

`/clock` and `/ground_truth/odom` are no bridges: they exist only in the
simulation and simply go away on the real robot.

## Hardware protocols

The emulated boards and their drivers talk over the wire protocols described
in [docs/protocols](docs/protocols):

| Board | Bus                     | Document                                       |
| ----- | ----------------------- | ---------------------------------------------- |
| IMU   | classic CAN, 500 kbit/s | [docs/protocols/imu.md](docs/protocols/imu.md) |
| RPLIDAR A1 | UART, 115200 baud | [docs/protocols/rplidar-a1.md](docs/protocols/rplidar-a1.md) |

## Emulated RPLIDAR A1

The simulator also exposes a standard-SCAN RPLIDAR A1M8 device through a
raw virtual UART. Its startup log names the allocated `/dev/pts/N` port.
A client can query information and health, start SCAN or FORCE_SCAN, read
five-byte angle/distance packets, and stop or reset the device. The
[protocol document](docs/protocols/rplidar-a1.md) includes a bounded Python
client and the exact emulated profile.

The emulator samples the live MuJoCo scene sequentially at a 476-microsecond
standard period, with a continuously powered 5.5 Hz motor and a 0.15–12 m
range. Samples share the current world state within a physics step. New
acquisitions pause with the simulation; commands and queued serial output
continue to be serviced. UART delivery is paced to 115200-baud 8N1 in wall
time, with a bounded backlog and explicit failure if a host cannot keep up.

The separate `lidar.py` driver consumes this serial path and is the sole
ROS `/scan` publisher. The display and guard retain their direct raycasts.
Express scan modes and USB adapter motor-control lines are
outside the standard-SCAN emulator.


The workshop launch passes the same `lidar_port` to simulator and driver.
Its default is `$HOME/.ros/robocell-lidar`, an exclusive symlink created by
the simulator and removed on normal shutdown. Override the launch argument
for concurrent instances; the simulator refuses to overwrite an existing
endpoint. After an unclean crash, remove only a stale link belonging to the
stopped simulator before restarting.

```console
ros2 launch robocell_sim workshop.launch.xml
ros2 topic echo --once /scan
```

For separate processes, start the simulator and then the driver with the
same explicit endpoint (the default is shared automatically):

```console
ros2 run robocell_sim main.py
ros2 run robocell_sim lidar.py --ros-args -p use_sim_time:=true
```

Driver parameters are `port`, `frame_id` (`base_scan`), `range_min` (0.15 m),
`range_max` (12.0 m) and `connection_timeout` (10.0 s). Startup sends STOP,
drains stale bytes, checks health, queries sample timing and sends SCAN.
Protocol errors, a disconnected terminal or missing data while ROS time
advances restart the session with bounded transactions and reconnect waits.
Pausing simulated time keeps the connection and gives it a fresh grace
period on resume. Backwards time restarts acquisition. Shutdown sends STOP.
