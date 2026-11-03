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
Stable angular resampling for SLAM is separate work under #148. The
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
