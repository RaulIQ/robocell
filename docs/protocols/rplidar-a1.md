# UART protocol of the RPLIDAR A1

This document is the contract for the lidar emulator and its ROS driver.
It describes the A1 standard SCAN protocol, with the original bytes and
units. The emulator will obtain distances from MuJoCo; the driver will know
only the serial stream. Neither side adds timestamps, sequence numbers,
checksums or custom headers to measurement packets.

The simulator exposes the standard-SCAN emulator on a pseudo-terminal.
A separate ROS driver consumes the stream and alone publishes `/scan`.
Express scan modes are separate formats
and are not described here; standard SCAN compatibility does not imply
support for every A1 firmware feature or SDK startup sequence.

## Sources

The wire format follows SLAMTEC's
[Interface Protocol and Application Notes](https://wiki.slamtec.com/download/attachments/83066883/LR001_SLAMTEC_rplidar_protocol_v2.4_en.pdf)
for the A and S series, particularly the standard SCAN, device information,
health and sample-period sections. The
[SDK command definitions](https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/include/sl_lidar_cmd.h)
and [response framing](https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/include/sl_lidar_protocol.h)
provide the corresponding packed structures. The official
[A1 ROS 2 launch configuration](https://github.com/Slamtec/sllidar_ros2/blob/main/launch/sllidar_a1_launch.py)
uses 115200 baud.

## Serial connection

Use a binary, bidirectional UART stream at 115200 baud, eight data bits,
no parity and one stop bit, with no software flow control. Multi-byte
integers are unsigned and little-endian unless a field says otherwise.
On the real device, use the specified electrical interface or its USB
adapter. A Linux pseudo-terminal substitutes for the UART in simulation;
it must operate in raw mode, without echo or newline translation.

A read may contain part of a packet or several packets. A write may be
partial. Neither side treats operating-system read or write boundaries as
protocol boundaries.

Motor power and rotation control are distinct from the SCAN command.
The A1 adapter's motor-control line is not a field in the measurement
stream. A virtual port does not reproduce physical motor control merely
by accepting UART bytes; that behavior must be handled explicitly when
implementing and checking compatibility with a real-device driver.

## Requests and responses

Each command below is exactly two bytes, `A5` followed by the command.
These commands have no payload and no request checksum.

| Request | Bytes | Response |
| ------- | ----- | -------- |
| STOP | `A5 25` | No response; leave scanning |
| RESET | `A5 40` | Reboot the core; no framed response |
| SCAN | `A5 20` | Descriptor, then standard measurement packets |
| FORCE_SCAN | `A5 21` | Same measurement format, bypass rotation-stability check |
| GET_INFO | `A5 50` | Descriptor, then 20 information bytes |
| GET_HEALTH | `A5 52` | Descriptor, then three health bytes |
| GET_SAMPLERATE | `A5 59` | Descriptor, then four sample-period bytes |

GET_SAMPLERATE requires firmware supporting that command, introduced in
version 1.17. Its name refers to sample periods in microseconds, not hertz.
Commands with payloads use additional framing and are outside this
standard-SCAN contract; they must not be parsed as two-byte requests.

A response begins with a seven-byte descriptor:

| Offset | Length | Meaning |
| ------ | ------ | ------- |
| 0 | 2 | `A5 5A` |
| 2 | 4 | Payload size in the lower 30 bits, send mode in the upper two |
| 6 | 1 | Response type |

Send mode 0 means one payload; mode 1 means a continuous stream of payloads.
The size is for one payload, not a complete rotation. Check size, mode and
type together before interpreting the following bytes.

| Request | Descriptor |
| ------- | ---------- |
| SCAN / FORCE_SCAN | `A5 5A 05 00 00 40 81` |
| GET_INFO | `A5 5A 14 00 00 00 04` |
| GET_HEALTH | `A5 5A 03 00 00 00 06` |
| GET_SAMPLERATE | `A5 5A 04 00 00 00 15` |

GET_INFO returns model (one byte), firmware minor (one byte), firmware major
(one byte), hardware revision (one byte), and serial number (16 bytes).
An emulator's identity must be documented as synthetic rather than copied
from an actual device. Its reported firmware must match supported commands.

GET_HEALTH returns status (one byte) followed by an error code (two bytes).
Status 0 means good, 1 warning and 2 error. A healthy response is
`A5 5A 03 00 00 00 06 00 00 00`.

GET_SAMPLERATE returns the standard sample period (two bytes) followed by
the express sample period (two bytes), both in microseconds. These values
must describe the selected firmware and actual operating profile, not
invent a rate for a mode that has not been implemented.

## Standard measurement packet

After the SCAN descriptor, each measurement occupies five bytes. There is
no descriptor between measurements or at rotation boundaries.

| Offset | Bits | Meaning |
| ------ | ---- | ------- |
| 0 | 0 | `S`: first measurement of a new rotation |
| 0 | 1 | Inverse of `S` |
| 0 | 2–7 | Quality, an unsigned six-bit value |
| 1 | 0 | Check bit, always 1 |
| 1 | 1–7 | Lower seven bits of `angle_q6` |
| 2 | 0–7 | Upper eight bits of `angle_q6` |
| 3 | 0–7 | Lower eight bits of `distance_q2` |
| 4 | 0–7 | Upper eight bits of `distance_q2` |

For bytes `b0` through `b4`, decode as follows:

```text
start       = b0 & 1
inverse     = (b0 >> 1) & 1
quality     = b0 >> 2
check       = b1 & 1
angle_q6    = (b1 | (b2 << 8)) >> 1
angle_deg   = angle_q6 / 64
distance_q2 = b3 | (b4 << 8)
distance_m  = distance_q2 / 4000
```

`start` and `inverse` must differ, `check` must be 1, and the angle must
be in `[0, 360)` degrees. A zero distance means an invalid return. It is
not a zero-distance obstacle. Quality describes return strength; it is
not a distance variance. The standard packet contains no CRC, checksum,
measurement index or acquisition time. Structural checks cannot detect
every corrupted distance or angle.

The A1 angle increases clockwise as viewed from above. ROS measures
positive angles counterclockwise around positive z. For a mounting whose
A1 zero direction coincides with `base_scan` positive x, the conversion is
`angle_ros = -angle_deg * pi / 180`, modulo one turn. Any mounting offset
must be applied explicitly; converting the sign must not mirror the room.

## Rotation boundaries and recovery

`S=1` starts a rotation, including that packet. It closes the preceding
rotation; it is not its last measurement. `S=0` continues the current
rotation. Discard data before the first accepted start marker after a
connection or recovery. The first start after SCAN need not be at exactly
zero degrees, and a real A1 need not produce 360 measurements per turn.
The emulator marks its first sample immediately even if the powered motor
is partway through a turn, so discard the initial partial rotation when
checking complete-circle counts.

Keep incomplete packets buffered. When structural checks fail, discard one
byte and search again, rather than skipping five bytes from an unknown
alignment. Require several consecutive plausible packets to regain
confidence, then wait for a start marker. Do not search for `A5 5A` inside
a healthy measurement stream: those values can occur in measurement data.

A driver must not silently join portions from different rotations after
losing alignment. Discard the damaged rotation and resume at a confirmed
boundary. Bounded buffers, timeouts and recovery diagnostics belong to the
driver. If recovery fails, STOP, drain stale input, and start a fresh SCAN
session with a validated descriptor. A missing point cannot be counted
exactly from this protocol because there is no sequence number.

## Session lifecycle

The core starts idle. The host queries health and information before
starting SCAN. SCAN sends one descriptor and begins producing measurements
when rotation is stable; FORCE_SCAN bypasses that stability check. Sending
SCAN while already scanning starts a new scan session and descriptor.

STOP stops measurement output but does not acknowledge it. Wait at least
one millisecond before another request, as specified by SLAMTEC, and
account for bytes already buffered in transit. Do not interpret those
stale bytes as a new response. RESET likewise has no framed acknowledgement;
startup output must not be treated as scan measurements. Allow bounded
reboot time before querying health again.

Query commands interrupt scanning, so the driver should STOP and drain
before a query and issue SCAN again afterwards. An error health status
requires recovery rather than publishing old scans as current data.

## Acquisition and delivery time

There are no custom timestamp bytes. The driver estimates scan timing
from reception and observed rotation boundaries. Serial buffering,
scheduling and polling introduce uncertainty; receiving a start packet
does not reveal the exact time its ray was measured.

The emulator must acquire successive rays from successive world states
according to MuJoCo time. Taking a whole instantaneous circle and delaying
its bytes does not reproduce a rotating lidar on a moving robot. If a
physics step spans several acquisitions, their approximation to the world
state must be documented and bounded by the physics step. Rotation and
new acquisitions stop while simulated time is paused; already queued bytes
may still be delivered.

At 115200 baud with 8N1, a byte takes ten wire bits. Five-byte packets
therefore allow at most 2304 measurements per wall-clock second before
other traffic. For example, 360 points at 5 turns/s would use 90000 bit/s,
about 78 percent of the link. This is a throughput example, not a claim
about A1 sample density or a fixed 360-point protocol requirement.

A pseudo-terminal does not enforce the baud rate. Document whether the
emulator also paces delivery to match the physical link. Faster-than-real
simulation cannot both exceed that wall-clock bandwidth and preserve real
UART timing. Simulation clock and serial delivery clock must not be
silently substituted for one another.

The emulator uses the A1M8-R5 distance interval, 0.15–12 m, and the typical
5.5 Hz rotation rate from the [A1M8 datasheet](https://bucket.download.slamtec.com/7fe7e3656e811ab1a645753af40809f05fa7ddcd/LD108_SLAMTEC_rplidar_datasheet_A1M8_v2.4_en.pdf).
Its standard period is 476 microseconds, the SDK's
[legacy standard-SCAN fallback](https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/include/sl_lidar_driver.h).
This produces about 2101 samples/s and 382 points/rotation, not a fixed
360 points. The 125-microsecond express period in GET_SAMPLERATE describes
the A1M8 hardware mode; Express packets are not implemented.

GET_INFO uses model 0x18, synthetic firmware 1.17, hardware field 5 and the
16-byte serial `ROBOCELL-A1-0001`. This identifies the emulated wire profile;
it is not a claim to reproduce a particular physical board's firmware.
The motor is modeled as continuously powered and already at 5.5 Hz, as if
wired to a fixed supply. SCAN and STOP control acquisition, not motor power.
PTY modem-control ioctls do not emulate the USB adapter's DTR motor line.
Quality is 15 for a valid return and 0 for an invalid one. The model has
ideal distances apart from protocol quantization, not a reflectivity or
optical-noise model. Display and guard raycasts retain separate 360-ray, 5 Hz, 0.12–3.5 m
settings; they do not publish ROS scans.

The simulator services acquisitions after every physics step and services
commands and delivery on every viewer iteration, including while paused.
At the current 2 ms physics step, several rays share one world state; each
measurement uses its own scheduled angle and acquisition is quantized by
at most one physics step. No full-circle snapshot is reused.

Delivery is paced by monotonic wall time at 11520 bytes/s, with at most
256 bytes of accumulated credit. Partial writes retain their unsent tail.
An application backlog above 65536 bytes fails explicitly; the kernel PTY
buffer is an additional finite queue. Commands already received can still
leave stale output in transit after STOP, as on a buffered real link.
A session advance requiring more than 4096 catch-up samples also fails
rather than allocating an unbounded output. Incomplete host requests
expire after five seconds of simulated device time. Unknown commands and
complete unsupported payload requests are consumed without a response.
RESET immediately returns this idealized board to idle without a startup
banner or a modeled reboot delay.

## ROS output

Publish one completed rotation as `sensor_msgs/LaserScan` in `base_scan`
on `/scan`. Convert millimeters to meters and invalid distances to `inf`.
Configure range limits for the selected device profile. Quality may be
published as device-specific intensity; it must not be called covariance.

The [ROS LaserScan definition](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg)
defines the header timestamp as the acquisition time of the first
array element, `time_increment` as the interval between elements, and
`scan_time` as the interval between scans. In this protocol, those times
are estimates, not device timestamps. Use simulated ROS time in simulation
and preserve timing uncertainty when evaluating the driver.

Clockwise acquisition becomes decreasing ROS angles. Keeping acquisition
order therefore requires a negative angular increment. If a driver
reorders points into increasing ROS angles, it must also account for the
changed first-ray time and temporal direction; reversing the array while
keeping the original start time and positive time increment misstates
when rays were acquired. Consumer compatibility and any timing
approximation must be checked when implementing the ROS publisher.

## Reference bytes

These are constructed packet examples, not recordings from physical
hardware. Quality is 15 for the valid returns below.

| Meaning | Five measurement bytes |
| ------- | ---------------------- |
| New rotation, 0 degrees, 1 m | `3D 01 00 A0 0F` |
| Same rotation, 1 degree, 1 m | `3E 81 00 A0 0F` |
| Same rotation, 90 degrees, 1 m | `3E 01 2D A0 0F` |
| Same rotation, 359 degrees, 3.5 m | `3E 81 B3 B0 36` |
| New rotation, 0 degrees, invalid return, quality 0 | `01 01 00 00 00` |

For 90 degrees, `angle_q6 = 5760`, and the packed angle with check bit is
`11521 = 0x2D01`, transmitted `01 2D`. For 1 m, `distance_q2 = 4000`,
transmitted `A0 0F`. Bytes `3C 01 00 A0 0F` must be rejected because the
start bits are equal; `3E 00 2D A0 0F` must be rejected because the check
bit is zero.

A minimal exchange is:

```text
host -> lidar: A5 20
lidar -> host: A5 5A 05 00 00 40 81
lidar -> host: 3D 01 00 A0 0F
lidar -> host: 3E 81 00 A0 0F
               ... remaining measurements ...
lidar -> host: 3D 01 00 A0 0F
host -> lidar: A5 25
```

The second start packet closes the preceding circle and belongs to the
next one. No extra header, timestamp or terminator is appended.

## Open the virtual device

Start the simulator normally. Its log prints the allocated slave path:

```text
RPLIDAR A1 standard SCAN is available on /dev/pts/N at 115200 baud
```

The actual number changes between runs. The simulator also binds an
explicit stable endpoint, `$HOME/.ros/robocell-lidar` by default, which the
workshop launch shares with the driver through its `lidar_port` argument.
Only one host should use the endpoint; do not run the manual client while
the ROS lidar driver is connected.
The simulator retains both endpoints until its viewer closes, so the port
exists even before a client connects. External client disconnects do not
implicitly issue STOP; send STOP before closing a session.

The following standard-library client queries health and reads one raw
measurement. Replace the example path with the logged path and let the
simulation run; acquisition does not advance while paused.

```bash
python - /dev/pts/N <<'PY'
import os
import select
import struct
import sys
import time
import tty


def exact(port, count):
    deadline = time.monotonic() + 2.0
    payload = b""
    while len(payload) < count:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([port], [], [], remaining)[0]:
            raise TimeoutError("RPLIDAR response did not arrive within two seconds")
        payload += os.read(port, count - len(payload))
    return payload


with os.fdopen(os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK),
               "rb+", buffering=0) as device:
    port = device.fileno()
    tty.setraw(port)
    try:
        os.write(port, bytes.fromhex("A5 52"))
        print("health:", exact(port, 10).hex(" "))
        os.write(port, bytes.fromhex("A5 20"))
        print("descriptor:", exact(port, 7).hex(" "))
        flags, angle, distance = struct.unpack("<BHH", exact(port, 5))
        print("start:", flags & 1, "quality:", flags >> 2,
              "angle:", (angle >> 1) / 64, "meters:", distance / 4000)
    finally:
        os.write(port, bytes.fromhex("A5 25"))
PY
```

This tests the device side only. It neither publishes ROS messages nor
replaces the `/scan` publisher. A host that starts a new session on a reused
port must first STOP, allow it to take effect, and drain stale input.


## Compatibility check

The official SLAMTEC SDK at commit
[`99478e5`](https://github.com/Slamtec/rplidar_sdk/commit/99478e5)
was used as an independent serial client against the emulator while MuJoCo
advanced a moving robot. Device information, health and
`startScan(false, false)` succeeded. After discarding the startup partial
rotation, `grabScanDataHq` returned three complete rotations of 382 points
at the reported 476-microsecond standard period, including valid returns.
This checks standard SCAN; the SDK's typical/Express mode and the complete
ROS driver startup sequence were not exercised.


## ROS driver behavior

`lidar.py` opens the explicit endpoint in raw 115200-baud 8N1 mode. It sends
STOP, waits, drains until a quiet interval, requests health and sample
period, and starts standard SCAN. Transactions are limited to one second.
The node publishes `/scan` on simulated time in `base_scan`; it imports no
MuJoCo. The simulator no longer opens a ROS scan publisher.

The receiver preserves partial packets and validates their inverse start
bit, check bit and angular interval. Losing alignment discards the damaged
turn and requires fresh boundaries, including one discarded partial turn,
before publication resumes. Receive buffers are bounded at 65536 bytes and
a rotation at 4096 measurements. Standard packets have no sequence counter
or checksum, so silent loss of a whole valid packet or plausible corrupted
payload cannot always be detected.

Each completed turn uses one uniform bin per received point, anchored at
its first angle. Clockwise points remain in chronological order. Therefore
`angle_increment` is negative, `angle_max` is numerically below `angle_min`,
and `time_increment` is positive. Binning absorbs Q6 angle quantization;
empty bins and zero-quality or out-of-range returns become `inf`. Quality
is copied to intensities. Binning is an angular approximation, not motion
distortion compensation.

`scan_time` is the point count times the reported standard sample period,
and `time_increment` is that period. Within a received chunk, first-ray
arrival estimates are backdated by the number of later packets and their
period. The completed turn's first stamp is no later than its closing
marker minus the estimated turn duration. These estimates cannot remove
unknown UART/USB buffering latency or variable motor/sample timing. They
retain temporal direction instead of reversing points with an unchanged
start stamp. No device timestamps are invented or added to the wire.

The liveness check uses monotonic wall time but observes ROS clock
progress. A two-second data gap triggers recovery while ROS time advances;
a paused clock does not. Resume starts a new grace period. A backwards ROS
clock or terminal hangup restarts the session. Failed operations reconnect
with 100 ms backoff under `connection_timeout`; a healthy long-running
session receives a fresh recovery budget. An orderly stop sends STOP even
when the polling loop exits because of ROS shutdown.

The named link is created exclusively and removed with its owning
simulator context. Its parent directory is created if needed. A second
simulator must use a different explicit endpoint. An unclean process crash
can leave a stale symlink; the simulator will fail rather than replace an
existing owner's path.

## Driver validation

A headless ROS 2 integration run used an asymmetric MuJoCo scene with a
moving robot, the real pseudo-terminal emulator, and a separate `lidar.py`
process on simulated time. It produced 21 scans. The ROS graph contained
one `/scan` publisher, named `lidar`. Runs exercised both 381- and 382-point
turns, a three-second pause, resume, terminal disconnection, a replacement
endpoint and clean shutdown. The recorded command stream ended with STOP.

Comparing sampled published rays against independently restored MuJoCo
poses gave a median absolute range error of 0.2 mm and a 95th percentile
of 1.2 mm in the final run. These figures validate this idealized scene and
timing path, not a real A1's optical accuracy. ROS `LaserProjection`
accepted the decreasing angular grid and returned the expected valid-point
count in `base_scan`.

The final local recording is under
`log/lidar-126/serial-scan-1793284132`, with 21 `/scan` messages and `/clock`.
The log directory is ignored by Git. To record another workshop run:

```console
ros2 bag record -o log/lidar-uart /scan /clock /tf /tf_static
```

The full GUI workshop was not run in the validation environment: it had no
display and no `vcan0` interface. A visual RViz check while driving remains
necessary on the configured development host. The headless scene comparison
and `LaserProjection` check do not substitute for that visual check.
