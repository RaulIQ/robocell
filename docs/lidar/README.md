# A1 acquisition timing across UART reads

Issue #147 exposed a timestamp regression when the ROS clock paused while
UART bytes remained queued. Backdating each read independently could give
later rotations earlier first-ray timestamps. The regression's three
stamps were `9998096000`, `9996192000` and `9998096000` nanoseconds.

The receiver now anchors the first complete turn and advances subsequent
turns by their sample count and the negotiated sample period. A chronology
reserves nonoverlapping intervals; it accepts a turn only when its closing
boundary is no later than the current ROS clock. A turn that cannot fit is
discarded with a warning. That invalidates the arrival anchor, while
preserving the end of the last published interval as a lower bound for
recovery. Damaged packets also invalidate the anchor. Queued valid turns
can still arrive during a pause; neither a pause nor a resume substitutes
wall time or the latest arrival time for old acquisitions.

The original `/scan` geometry remains chronological and clockwise, with
negative `angle_increment` and positive `time_increment`. Turns retain
variable point counts, invalid returns and their device-specific quality.
The fixed-grid SLAM adapter described below is separate from this raw
acquisition contract. The
standard A1 stream has no device timestamps or sequence numbers: the
sample-period estimate cannot eliminate buffering latency, clock error,
or every silently lost valid packet.

## Verification

The offline tests reproduce the original frozen-clock regression, vary
UART block sizes from one byte to the complete stream, check arrival
jitter and different turn lengths, and verify recovery without reusing
published intervals. Existing real pseudo-terminal tests cover the
serial-to-ROS path and continue to verify ray ordering and invalid ranges.

```console
python -m pytest --eo
```

`record.py` launches the actual headless workshop in a separate ROS domain
and records `/scan`, `/clock` and the motor and gyro readiness states in a
SQLite bag. Use an unused SocketCAN channel; it refuses existing traffic
and output directories. It waits for stationary gyro calibration, pauses
and resumes the simulator three times, and stops the lidar process for
1.1 wall seconds before the second pause so UART bytes accumulate. The
reader resumes while the simulation clock stays frozen. Every startup,
service, resumed-stream and shutdown wait is bounded.

```console
source /opt/ros/jazzy/setup.bash
python -m colcon build --symlink-install --packages-select robocell_sim
source install/setup.bash
python docs/lidar/record.py log/lidar-147/verification --domain 84
```

The script checks a frozen clock during pause, that drained turns end no
later than that clock, recovery to fresh acquisitions after each resume,
nonoverlapping timestamps across the entire recording, preserved A1
geometry and sample period, and exactly one `/scan` publisher. The resume
latency measures collection of six new scans, rather than first-message
latency. Process logs, scan intervals, check results and a bag digest remain
in the selected output directory. The [results](results.json) identify the
recordings used for this change. This validates timing consistency and
pause recovery; it does not establish exact acquisition-time accuracy on
physical hardware.

## Fixed-grid SLAM input (issue #148)

`mapping.launch.xml` starts `scan.py` alongside the original driver and
points SLAM Toolbox at `/scan/slam`. The workshop launch and `/scan` remain
available with the original clockwise acquisition geometry. The adapter
never modifies or republishes the input message.

The adapter transforms each finite, in-range endpoint from its individual
acquisition time through `odom` into `base_scan` at the last ray's estimated
acquisition time. TF supplies interpolated translations and rotations,
including the sensor offset. This compensates motion within a turn before
choosing angular bins; it does not obtain a new acquisition timestamp from
the scan's arrival time. All endpoint transformations must succeed before
the output is published.

The default grid has 360 bins with centers `-pi + index * 2*pi/360`, inclusive
endpoints `-pi` and `pi - 2*pi/360`, and positive `angle_increment`. The output
header identifies the common last-ray reference pose; `time_increment` is
zero because this is a deskewed snapshot. `scan_time` retains the raw turn's
sample-count duration. There is no claim that ascending output bearings
represent chronological acquisition.

Each point selects the nearest angular center, with half-bin ties going
toward the increasing index and wrapping across the seam. A collision keeps
the shortest projected range and its intensity; equal ranges retain the
first acquisition. NaN, infinite, nonpositive and out-of-range input returns
cannot occupy a bin. Projected ranges outside the reported limits are also
discarded. Empty bins contain positive infinity with zero intensity. The
adapter does not fill holes or interpolate returns across obstacles.

The ROS parameters are `input=/scan`, `output=/scan/slam`, `fixed_frame=odom`,
`bins=360`, `transform_timeout=0.5` and `capacity=8`. Keep the grid constant
throughout a SLAM session because SLAM Toolbox caches the first scan's
metadata. The queue retries unavailable TF without blocking ROS callbacks.
Its deadline uses monotonic wall time, so expiration still works while
simulation time is paused. Expired rotations and queue overflow are
reported and discarded; late TF cannot resurrect an expired rotation.

The approximation is planar: transformed endpoints are projected into the
reference sensor's XY plane and quantized to the nearest bin. Half a bin
at 12 m can shift an endpoint by about 10.5 cm. The input already has its
own angular binning, and its first-ray timestamps are estimates. TF and
odometry errors, device buffering and the emulator's 2 ms physics step
remain limitations. Deskewing cannot repair unknown acquisition-time
errors or observe surfaces hidden during the original measurements.

### UART delivery regression

The first moving integration run exposed growing raw-scan age, from about
0.24 s to 1.56 s, which correctly inhibited movement through the existing
0.5 s collision-guard freshness limit. Tracing showed the emulator's UART
backlog growing during slow physics and rendering iterations. Its credit
cap discarded elapsed transmission time even for bytes already queued.

The corrected delivery preserves elapsed wire time for the previous
backlog, retains the 11520 bytes/s rate and 256-byte idle-credit cap, and
keeps the existing backlog limit and partial-write behavior. A failing
regression reproduced only 255 transmitted bytes when 800 queued bytes
were already due. Additional tests check the baud limit with a backlog and
integer-budget rounding on long-running monotonic clocks. The raw scan's
angular and timestamp algorithms and the collision guard are unchanged.

### Reproduction and evidence

```console
source /opt/ros/jazzy/setup.bash
python -m pytest --eo
python -m colcon build --symlink-install --packages-select robocell_sim
source install/setup.bash
python docs/lidar/isolated.py log/lidar-148/isolated --domain 86
python docs/lidar/compatibility.py log/lidar-148/mapping --domain 90
```

Use fresh output directories and an unused SocketCAN channel (`--channel`
selects it). The isolated test exercises the installed adapter using actual
ROS topics and TF: alternating 381/382-point scans, shifted first angles,
delayed transforms, expiration with a frozen `/clock`, and fresh recovery.
The full test launches the installed mapping stack, waits for readiness,
commands a stationary turn and a short arc, stops and resumes the EKF for
0.2 s and 1.0 s, and pauses and resumes the simulator twice. Every wait and
process shutdown is bounded. It records both scans, clock, TF, filtered
odometry, ground truth, commands, SLAM poses and occupancy maps in a bag.

The full test checks unchanged raw geometry, one raw publisher, a fixed
adapted grid, monotonic reference stamps, raw-scan freshness, actual robot
rotation, successive SLAM pose stamps matching adapted scans, map updates,
and the absence of SLAM point-count rejection messages. Separate unit
tests check asymmetric obstacle bearings, motion compensation, invalid
returns, collisions and complete input-message preservation.

Geometry diagnostics express adapted endpoints in the workshop's world
using the next ground-truth pose at the reference timestamp (at most one
20 ms reporting interval later). They measure distance to the nearest
static box or cylinder surface at the endpoint's height. Truth is used
only for this diagnostic, never as an adapter or SLAM input. These are
surface residuals, not localization or map-accuracy measurements. See
[compatibility-results.json](compatibility-results.json) for measured
errors, scan ages, installed SLAM version, checks and bag digests. Saved-map
validation, route accuracy and loop closure remain in issue #136.
