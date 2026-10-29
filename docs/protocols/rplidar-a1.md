# UART protocol of the RPLIDAR A1

This document is the contract for the planned lidar emulator and driver.
It describes the A1 standard SCAN protocol, with the original bytes and
units. The emulator will obtain distances from MuJoCo; the driver will know
only the serial stream. Neither side adds timestamps, sequence numbers,
checksums or custom headers to measurement packets.

This is a specification, not an implemented device. The current simulator
still publishes `/scan` directly. Express scan modes are separate formats
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

The current 360-ray, 5 Hz, 0.12–3.5 m simulator settings are not the hardware
specification of an A1. Before implementing the emulator, select the A1M8
hardware/firmware profile from its datasheet and set the sample period,
rotation rate and range limits accordingly. Packet encoding alone does
not reproduce hardware noise, reflectivity or motor dynamics.

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
