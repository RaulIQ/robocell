# CANopen protocol of the wheel drives

This is the implementation contract for issue [#135](https://github.com/RaulIQ/robocell/issues/135).
It specifies two synthetic wheel drives on classic CAN, with a bounded subset
of CANopen CC and CiA 402. It is not a physical drive datasheet or a claim of complete CANopen or
CiA 402 conformance. The standalone emulator described below implements the
device side; the host and workshop integration are described below.

Each emulator receives CAN frames, controls one MuJoCo wheel actuator, and
acquires that wheel's encoder and velocity. The emulators contain no ROS.
A separate host driver knows the protocol and kinematics but imports no
MuJoCo. A CANopen stack may supply communication services; the application
still implements the drive states, watchdog and physics boundary below.

## Sources and limits

The starting references are CiA's descriptions of
[CiA 402 drives and motion control](https://can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control),
[network management](https://www.can-cia.org/can-knowledge/network-management/),
and [error control](https://www.can-cia.org/can-knowledge/error-control-protocols).
They explain the distinction between drive control, communication state and
heartbeat. The units, mappings, timing limits and manufacturer objects below
are choices for this synthetic profile, not requirements inferred from those
overviews. They must be checked against the selected stack during implementation.

Only the synthetic profile-velocity subset (PV, mode 3) is supported.
PDOs are asynchronous and SYNC is not used. The existing MuJoCo velocity
actuator handles physical response; configurable profile acceleration and
deceleration objects are not implemented. This is not a complete PV model. Position, torque, homing, CAN FD, dynamic PDO remapping,
segmented or block SDO, LSS, node guarding, persistent configuration and
CANopen Safety are outside this profile. Quick-stop is a simulated stop,
not a certified E-STOP or STO function. Wheel geometry, collision-guard
geometry, SLAM and localization are outside this work.

## Bus and identities

Use classic CAN data frames, standard 11-bit identifiers, at most eight data
bytes and little-endian integers in two's complement when signed. Reject
extended, remote, error and FD frames as application input. SocketCAN
`vcan0` is the default shared interface; an isolated virtual interface is
required for integration tests. A physical bus would use 500 kbit/s;
virtual CAN does not reproduce arbitration timing or electrical faults.

The left and right node IDs default to 1 and 2. Both are startup parameters,
must be distinct and in 1..127, and cannot change while running. Reserve
host node 127 for heartbeat; neither drive may use it. Exactly one host
owns motion commands and NMT on the bus. Configuration fails before opening
actuators if IDs conflict. The existing IMU frames `0x101` and `0x102` do not
collide with these mappings.

| CAN identifier | Direction | Length | Purpose |
| --- | --- | --- | --- |
| `0x000` | Host to drives | 2 | NMT command, target node |
| `0x080 + n` | Drive to host | 8 | EMCY |
| `0x180 + n` | Drive to host | 8 | TPDO1: status, position, sample |
| `0x200 + n` | Host to drive | 6 | RPDO1: controlword, target velocity |
| `0x280 + n` | Drive to host | 6 | TPDO2: actual velocity, sample |
| `0x380 + n` | Drive to host | 8 | TPDO3: acquisition time |
| `0x580 + n` | Drive to host | 8 | SDO response |
| `0x600 + n` | Host to drive | 8 | SDO request |
| `0x700 + n` | Drive to host | 1 | Boot-up or heartbeat |
| `0x77F` | Host to drives | 1 | Host heartbeat, value `05` |

Here `n` is the configured drive ID. Frames addressed to other nodes are
ignored. CAN preserves frame boundaries: a short frame is malformed, not
the beginning of a frame to be completed by another receive call.

## Wheel units and signs

Commands and feedback describe the wheel shaft, after any gearing. Positive
rotation moves that wheel forward along the robot's positive x direction.
The emulator accounts for each MuJoCo joint axis so both positive wheel
commands produce forward travel. No motor-side gear ratio enters the wire.

Target and actual velocity are signed 32-bit integers in 0.001 rad/s.
The allowed target interval is -7880..7880, matching the existing 7.88 rad/s
wheel limit. Invalid targets are rejected, never silently clipped. Convert
actual velocity by rounding to the nearest integer, with ties away from zero.
An actual velocity outside the signed 32-bit range is a local acquisition
fault rather than a saturated measurement.

Position is signed 32-bit encoder counts at 4096 counts per wheel revolution:

```text
count = floor(wheel_angle_rad * 4096 / (2 * pi))
wire_position = ((count + 2^31) modulo 2^32) - 2^31
```

This preserves the existing encoder's floor quantization, including negative
angles. Position is absolute relative to the model joint's angle origin;
startup, NMT reset and fault reset do not zero the physical encoder.
The host unwraps consecutive positions using the signed modular difference.
A jump of exactly half the counter range is ambiguous and invalidates the
baseline. After a restart or feedback gap above 300 ms of simulated time,
establish a new baseline rather than integrating an unobserved interval.
This can lose travel during an outage; it must not create a fictitious jump.

The host converts counts to radians with `2*pi/4096`, and velocity to rad/s
with `/1000`. Keep the existing radius 0.033 m, separation 0.288 m and
arc-preserving wheel limit. Forward body commands give two positive wheel
speeds; a positive yaw command increases the right wheel relative to the left.

## Object dictionary and SDO

All listed subindex-0 objects are scalar unless described as arrays. RO means
read-only over SDO; the emulator can update its own feedback objects.
Configuration is volatile. RW configuration writes require NMT
pre-operational and CiA 402 Switch on disabled. The host uses RPDO1 for
cyclic commands; SDO writes of controlword or target velocity are unsupported.

| Index:subindex | Type | Access | Value or meaning |
| --- | --- | --- | --- |
| `1000:00` | UNSIGNED32 | RO | `0x00000192`, synthetic drive profile 402 |
| `1001:00` | UNSIGNED8 | RO | Error register, zero when healthy |
| `1016:00` | UNSIGNED8 | RO | One heartbeat consumer entry |
| `1016:01` | UNSIGNED32 | RW | `0x007F012C`: host 127, timeout 300 ms |
| `1017:00` | UNSIGNED16 | RW | Producer heartbeat, default 100 ms |
| `1400:00` | UNSIGNED8 | RO | RPDO1 communication record, highest subindex 2 |
| `1400:01` | UNSIGNED32 | RO | `0x200 + n` |
| `1400:02` | UNSIGNED8 | RO | Transmission type 255, asynchronous |
| `1600:00` | UNSIGNED8 | RO | Two RPDO1 mapping entries |
| `1600:01..02` | UNSIGNED32 | RO | `60400010`, `60FF0020` |
| `1800..1802:00` | UNSIGNED8 | RO | TPDO communication records, highest subindex 5 |
| `1800..1802:01` | UNSIGNED32 | RO | Respectively `0x180+n`, `0x280+n`, `0x380+n` |
| `1800..1802:02` | UNSIGNED8 | RO | Transmission type 255 |
| `1800..1802:03` | UNSIGNED16 | RO | Inhibit time 0 |
| `1800..1802:05` | UNSIGNED16 | RO | Event timer 20 ms of simulated time |
| `1A00:00` | UNSIGNED8 | RO | Three TPDO1 mapping entries |
| `1A00:01..03` | UNSIGNED32 | RO | `60410010`, `60640020`, `20000010` |
| `1A01:00` | UNSIGNED8 | RO | Two TPDO2 mapping entries |
| `1A01:01..02` | UNSIGNED32 | RO | `606C0020`, `20000010` |
| `1A02:00` | UNSIGNED8 | RO | Two TPDO3 mapping entries |
| `1A02:01..02` | UNSIGNED32 | RO | `20010120`, `20010220` |
| `2000:00` | UNSIGNED16 | RO | Manufacturer sample counter |
| `2001:00` | UNSIGNED8 | RO | Two acquisition-time fields |
| `2001:01..02` | UNSIGNED32 | RO | Low and high words of simulated nanoseconds |
| `2002:00` | UNSIGNED16 | RW | Command watchdog, default 200 ms |
| `2003:00` | UNSIGNED32 | RO | Profile revision 1 |
| `603F:00` | UNSIGNED16 | RO | Latched fault code, zero when healthy |
| `6040:00` | UNSIGNED16 | RO | Last accepted RPDO controlword |
| `6041:00` | UNSIGNED16 | RO | Statusword |
| `6060:00` | INTEGER8 | RW | Requested mode, only 3 accepted, default 3 |
| `6061:00` | INTEGER8 | RO | Active mode, 3 |
| `6064:00` | INTEGER32 | RO | Encoder position |
| `606C:00` | INTEGER32 | RO | Actual shaft velocity |
| `608F:00` | UNSIGNED8 | RO | Two encoder resolution entries |
| `608F:01` | UNSIGNED32 | RO | 4096 encoder increments |
| `608F:02` | UNSIGNED32 | RO | 1 wheel revolution |
| `60FF:00` | INTEGER32 | RO | Last accepted target; cleared on inhibition |
| `6502:00` | UNSIGNED32 | RO | `0x00000004`, mode 3 supported |

Mapping entries are shown as hexadecimal without the `0x` prefix. Unlisted
subindices, including holes in communication records, are unsupported.
Producer period must be 50..200 ms, heartbeat timeout 250..1000 ms with
node field 127 and reserved byte zero, and watchdog 100..500 ms. Zero cannot
disable supervision. The host reads back configured values before enabling.

Only expedited SDO uploads/downloads with explicit size are supported.
Requests and responses are eight bytes, with unused response bytes zero.
Upload request byte 0 is `40`; successful responses are `4F`, `4B`, or `43`
for 1, 2, or 4 bytes. Download requests use `2F`, `2B`, or `23`; the response
is `60`. Bytes 1..2 are the index, byte 3 the subindex, and bytes 4..7 the
value. Validate size, type, state and range before changing anything.

An abort starts with `80`, echoes index/subindex, and carries its UNSIGNED32
code in bytes 4..7:

| Code | Reason |
| --- | --- |
| `05040001` | Unsupported SDO command, including segmented/block transfer |
| `06010002` | Write to a read-only object |
| `06020000` | Unknown index |
| `06070010` | Wrong data length/type |
| `06090011` | Unknown subindex |
| `06090030` | Value out of range or unsupported mode |
| `08000022` | Write forbidden in the current state |

Discard malformed SDO frames without an answer when their length is not
8; no partial dictionary write is allowed. Permit one SDO request at a time
per node. Host response timeout is 250 ms monotonic time, with at most two
retries of an idempotent request within the overall startup budget.

## PDO contract

RPDO1 contains controlword (UNSIGNED16), then target velocity (INTEGER32).
Send every 20 ms of monotonic time while enabled, even for zero speed.
Only a complete, valid RPDO1 in NMT operational can refresh the watchdog.
Validate the whole frame atomically; an invalid target or controlword must
not change the state, target or watchdog deadline. Frames outside NMT
operational are ignored and do not refresh it.

On the transition into Operation enabled, discard the frame's target and
keep zero output. Only a later RPDO1 received while already enabled can
apply a velocity. Enabling also requires an unexpired operational host heartbeat
and an unpaused simulator. Disabled-state commands never queue motion for later.
NMT operational alone never enables movement.

Every 20 ms of simulated time in NMT operational, acquire one coherent
snapshot per wheel and transmit TPDO1, TPDO2, TPDO3 in that order:

| PDO | Layout |
| --- | --- |
| TPDO1 | statusword u16, position i32, sample u16 |
| TPDO2 | actual velocity i32, sample u16 |
| TPDO3 | acquisition time u64, low word first, in simulated nanoseconds |

Both sample fields match and advance modulo 65536. TPDO3 closes the snapshot.
No other snapshot for that node may interleave its frames. The host starts
assembly on TPDO1, accepts matching TPDO2, and commits only on TPDO3. Wrong
order, length, sample mismatch or a new TPDO1 discards the incomplete sample.
Expire partial assemblies after 100 ms monotonic time. A lost closing frame
must not attach the next sample's timestamp to old feedback. Other nodes'
frames and heartbeat/SDO/EMCY may interleave freely.

These asynchronous PDOs deliberately include manufacturer sample/time fields;
they are not a generic default CiA 402 PDO layout. The driver publishes joint
feedback only from complete snapshots. Pair left/right acquisition times
within one physics step; discard unmatched older samples after 100 ms.
Status changes are visible immediately through SDO; they appear in the next
scheduled PDO snapshot, without an extra snapshot at a duplicated timestamp.

## NMT and heartbeat

NMT is a communication state machine, independent of the CiA 402 drive state.
At power-on, finish initialization within 500 ms monotonic time, send one
boot-up byte `00` on `0x700+n`, then enter pre-operational. Thereafter send
heartbeat every configured producer period, including while stopped or paused:
`7F` pre-operational, `05` operational, `04` stopped. Boot-up is not a heartbeat.

| NMT command byte | Meaning |
| --- | --- |
| `01` | Start: operational, without enabling the drive |
| `02` | Stop: inhibit motion, stop PDO and SDO |
| `80` | Pre-operational: inhibit motion, allow SDO, stop PDO |
| `81` | Reset node: inhibit motion, restore volatile defaults and reboot |
| `82` | Reset communication: inhibit motion, restore 1xxx defaults and reboot |

Byte 1 is the target node, or zero for broadcast. Unknown commands and invalid
lengths are ignored. NMT and heartbeat remain active in stopped. EMCY and SDO
are available in pre-operational/operational only. A latched fault arising in
stopped is retained and reported when communication permits. Reboot sends a
new boot-up; reset communication preserves application configuration and
latched faults. Reset node clears faults only if their causes are absent.
Neither reset moves or zeros the physical wheel.

The host sends `05` on `0x77F` every 100 ms. Each drive starts host supervision
on entering NMT operational, with one full consumer-timeout grace interval.
Only a valid operational heartbeat from host 127 refreshes it; boot-up or a
non-operational host heartbeat inhibits motion immediately. The host declares
a drive unavailable after 300 ms without a valid heartbeat by default.
It also treats an unexpected boot-up or non-operational state during motion
as loss of that drive. A healthy heartbeat does not prove fresh commands
or feedback; these are supervised separately.

## CiA 402 drive states

The emulator starts in Not ready to switch on (`0000`), then reaches Switch
on disabled (`0040`) before boot-up. The table gives masked statusword state
patterns, not independent bit flags. Supported status bits are 0..6; other
bits are zero. Match states with mask `004F` for Not ready, Switch on disabled,
Fault reaction active and Fault, and `006F` for the remaining states.

| Current state | Accepted command | Next state | Status pattern |
| --- | --- | --- | --- |
| Switch on disabled | Shutdown `0006` | Ready to switch on | `0021` |
| Ready to switch on | Switch on `0007` | Switched on | `0023` |
| Switched on | Enable operation `000F` | Operation enabled | `0027` |
| Operation enabled | Disable operation `0007` | Switched on | `0023` |
| Switched on / Operation enabled | Shutdown `0006` | Ready to switch on | `0021` |
| Any initialized non-fault state | Disable voltage `0000` | Switch on disabled | `0040` |
| Operation enabled | Quick-stop `0002` | Quick-stop active | `0007` |
| Ready / Switched on | Quick-stop `0002` | Switch on disabled | `0040` |
| Any non-fault state | Local fault | Fault reaction active | `000F` |
| Fault reaction active | Next service cycle | Fault | `0008` |
| Fault | Fault reset `0080`, rising bit 7, cause absent | Switch on disabled | `0040` |

Only the exact listed controlwords are supported; reserved/mode-specific bits
must be zero. Repeating the current state's command is idempotent. A recognized
command in an inapplicable state leaves the state and target unchanged; it
cannot skip startup stages. For example, `000F` cannot enable a disabled drive.
Fault reset cannot clear an active cause, and a held reset bit cannot repeatedly
reset a fault. A subsequent controlword with bit 7 clear rearms reset detection.

Quick-stop immediately replaces the actuator velocity target with zero.
Keep Quick-stop active until measured speed is below 0.01 rad/s, then enter
Switch on disabled. Failure to stop within 1 s monotonic time latches a local
fault. The velocity actuator and MuJoCo dynamics determine deceleration;
this profile implements no configurable acceleration ramp or stop option code.
Disable, NMT inhibition and faults also write zero and clear stored targets.
Do not claim zero target means instantaneous zero physical velocity.

## Watchdogs, faults and recovery

Arm the command watchdog when entering Operation enabled and disarm it on
leaving that state; heartbeat supervision remains active. Expiry at the
configured interval without a valid RPDO1 immediately writes zero and latches
fault `FF01`. A driver's continued retransmission of its last command must
not conceal stale application input: the host independently expires body
commands after 250 ms monotonic time and renews zero targets at both wheels
while keeping the drives enabled. The first fresh application command can
resume motion without a new enable sequence; an expired command cannot be
replayed. Transport, heartbeat, feedback and drive faults still trigger the
protective stop and require verified enable before fresh motion.

Host heartbeat loss latches `8130`; a local acquisition, actuator or stop
failure latches `FF02`. On a fault, output is zero before any further physics
step. Fault reaction active lasts one service cycle before Fault. Set error
register `1001` to `11` for communication faults (`8130`, `FF01`), or `01`
for local faults, and expose the first latched code in `603F`. Preserve the
first code until reset; later causes still prevent reset while active.

EMCY has error code u16, error register u8 and five zero manufacturer bytes.
Send once per new latched fault when NMT permits. Successful fault reset
sends error code zero and clears the register. The command-timeout cause
becomes inactive when valid zero-target RPDOs arrive; heartbeat-timeout cause
requires restored operational host heartbeat. Clearing a local failure needs
its cause fixed and the standalone emulator restarted; CAN fault reset cannot
clear the local fault latch. Ordinary commands and
NMT start do not clear faults.

A drive loss, a feedback gap exceeding 300 ms of simulated time while that
clock advances, CAN error or local host failure makes the host stop both wheels. Attempt zero/quick-stop on the
reachable drive; the unreachable drive must stop through its own watchdog.
Startup, reconnection and enable must never wait indefinitely:

1. Open the configured CAN interface and start host heartbeat.
2. Discover both heartbeats, or request reset communication if no boot-up was
   observed; establish a new feedback epoch and discard buffered snapshots.
3. Put both nodes in pre-operational, verify profile revision, mode, units,
   mapping and timeout values through SDO, then start NMT operational.
4. Require fresh heartbeats. If faulted, obtain explicit operator authorization
   for fault recovery, verify the cause is absent, send zero with reset `0080`,
   then clear bit 7. Normal startup does not automatically clear a fault.
5. Walk `0006`, `0007`, `000F` with zero targets, confirming each state through
   SDO or complete feedback, with 500 ms per transition. State checks do not
   depend on the arrival of both nodes' boot-up or SDO responses together.
6. Require a body command received after successful recovery/enable before
   sending a nonzero target. Commands cached before the outage are discarded.

Bound one startup or recovery attempt to 5 s monotonic time. Retry connection
with 100 ms backoff within that budget; after expiry, remain stopped and
report failure until an explicit retry. A later failure gets a new budget.
A reset, disconnect, backwards simulation clock or unexpected boot-up invalidates
position baselines and partial snapshots. A stopped drive requires a full
verified enable sequence; restoring the bus never restores old velocity.

On clean host shutdown, send zero/quick-stop to both drives, allow at most
500 ms for status feedback, then send disable voltage and close the bus.
A missing acknowledgement cannot delay shutdown indefinitely; drive-side
watchdogs remain the fallback. Simulator shutdown writes both actuator targets
to zero before releasing resources.

## Acquisition, pause and service clocks

Encoder acquisition, sample numbering, timestamps and the 20 ms TPDO period
use simulated time. Acquire after a physics step from the actual joint
state; timestamp with that step's simulated time in nanoseconds. Quantization
is bounded by one physics step. Do not synthesize old acquisitions from the
current state during catch-up; skip missed periods and record the gap.

CAN service, startup limits, heartbeat, command age, watchdogs, quick-stop
timeouts and reconnect use monotonic wall time. Service CAN and protective
checks at least every 10 ms, including while physics is paused. Inhibit output
before the next physics step if a deadline expired. A stalled emulator process
cannot enforce a wall-time deadline while stalled; simulation stepping must
not continue independently of its protective checks.

On entering pause, the simulator immediately zeroes both targets, clears
pending motion and puts non-faulted drives in Switch on disabled. While paused,
ignore enable requests and all motion targets; continue NMT, SDO, heartbeat,
fault reset and zero-output protection. Do not acquire or send new TPDO samples.
Host commands still age in wall time. A frozen `/clock` can suspend only the
host's feedback-gap alarm, never heartbeat or command supervision.

On resume, the first feedback exposes the disabled state. The host discards
old commands and performs the verified enable sequence, then requires a fresh
body command before motion. Watchdog/heartbeat faults raised during pause
still require explicit recovery. A backwards acquisition timestamp starts a
new epoch; it must not become a negative odometry time interval.

## Host integration boundary

All command sources, including keyboard input, enter one explicit guarded
body-command path before conversion to wheel speeds. The existing collision
guard uses the existing scan geometry and must be reevaluated as scans arrive,
even if the requested body command stays unchanged. A guard stop supersedes
motion; guard data older than 500 ms of simulated time must inhibit motion.
Before the first scan, motion is inhibited. The command path must preserve
the original input's age so periodic guard output cannot keep an abandoned
body command fresh. No command source may write MuJoCo actuators directly.

Only drive emulators write wheel actuators and acquire encoder feedback.
The ROS host publishes exactly one `/joint_states` stream from matched CAN
snapshots, with acquisition time, positions in radians and velocities in rad/s.
The simulator's direct joint publisher is disabled on this path. Existing
wheel odometry and the EKF consume this feedback; neither imports MuJoCo.
Ground truth remains a separate simulator output. Startup and shutdown must
also work with a non-ROS CAN client.

## Reference frames

These are constructed test vectors for default left node 1, not recorded
hardware traffic. Spaces separate bytes; identifiers are hexadecimal.

```text
701  00                         boot-up, then pre-operational
701  7F                         pre-operational heartbeat
77F  05                         host heartbeat
601  40 03 20 00 00 00 00 00    read profile revision 2003:00
581  43 03 20 00 01 00 00 00    revision 1
601  2F 60 60 00 03 00 00 00    set mode 3 in pre-operational
581  60 60 60 00 00 00 00 00    successful download
000  01 01                      start node 1, still disabled
201  06 00 00 00 00 00          shutdown, zero target
201  07 00 00 00 00 00          switch on, after confirming ready
201  0F 00 00 00 00 00          enable, after confirming switched on
201  0F 00 E8 03 00 00          later frame: +1.000 rad/s
201  0F 00 18 FC FF FF          -1.000 rad/s
201  02 00 00 00 00 00          quick-stop
601  23 FF 60 00 E8 03 00 00    unsupported SDO target write
581  80 FF 60 00 02 00 01 06    abort 06010002
081  01 FF 11 00 00 00 00 00    command watchdog fault FF01
```

The startup excerpt omits intermediate state reads and repeated heartbeats;
a real client must wait for each state and keep supervision alive.
A healthy snapshot with sample 17, position 4096 counts, speed +1 rad/s and
acquisition at 1 s is:

```text
181  27 00 00 10 00 00 11 00
281  E8 03 00 00 11 00
381  00 CA 9A 3B 00 00 00 00
```

Position -1 is `FF FF FF FF`; rollover from `FF FF FF 7F` to
`00 00 00 80` means +1 count, not a reverse jump. Frames from right node 2
use identifiers incremented by one and an independently checked encoder sign.

## Implementation acceptance

Start with failing protocol and state-transition tests derived from this
contract. Exercise the host against independent emulator CAN endpoints on
an isolated virtual bus, with bounded waits and no Internet requirement.
Include both node IDs, straight travel, both turns, reverse, zero speed,
encoder quantization/signs/rollover, and coherent timestamped feedback.

Cover boot-up in either order, delayed startup responses, lost PDO fragments,
malformed lengths/flags, unsupported objects/modes/writes, invalid targets,
disabled commands, stale body commands, lost commands/heartbeat/feedback,
quick-stop, pause/resume, drive restart, clock reset, disconnect, explicit
fault recovery, bounded failed reconnect and clean shutdown. Verify stopping
at the actuator boundary, not just a statusword or published zero command.
Confirm motion cannot resume from a cached pre-failure command.

Before closing #135, document actual executable names and launch parameters,
record a repeatable route with CAN traffic and `/cmd_vel`, `/joint_states`,
`/wheel/odometry`, `/odometry/filtered`, `/ground_truth/odom` and `/clock`, and
check that the ROS graph has one joint-feedback publisher. Record watchdog
and quick-stop outcomes, pause behavior and startup/recovery commands.
The standalone checks below do not establish those ROS integration results
or claim that the workshop protocol path is already enabled.


## Standalone emulator

`robocell_sim.motors` runs the two wheel endpoints and MuJoCo on one physics
thread, with no ROS initialization or ROS imports. Each drive opens its own
CAN connection. The implementation uses `canopen==2.4.1` for typed object
storage, SDO responses, NMT state and periodic heartbeat, and `python-can`
for the transport. Drive permission, fault latching, fixed PDO payloads and
physics acquisition belong to the emulator. No notifier thread accesses
MuJoCo. A service call processes at most 256 queued frames; overflow fails
explicitly with zero output. The context closes heartbeat tasks and zeroes
its actuator even if startup or service raises an exception.

In the development environment, bring up `vcan0` on the host if it does not
already exist:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
```

From the repository root, start the model and monitor its bus in another
terminal:

```bash
PYTHONPATH=src/robocell_sim python -m robocell_sim.motors \
  --model src/robocell_sim/models/waffle_pi/scene.xml \
  --channel vcan0 --left-node 1 --right-node 2 --viewer

candump vcan0
```

Omit `--viewer` for headless operation. `--duration 10` bounds the run to ten
wall seconds. Space toggles pause in the viewer. The model argument is
explicit so the same command works for source and installed scripts; the
model must contain `wheel_left`/`wheel_right` velocity actuators and
`wheel_left_joint`/`wheel_right_joint` joints with forward-positive axes.

A non-ROS host must send its heartbeat, start NMT, verify the state after
each zero-target enable command, and then refresh command PDOs every 20 ms.
The reference frames above define the startup exchange; merely starting
this executable never moves the robot. Stop command renewal to exercise the
200 ms watchdog, or stop host heartbeat to exercise the 300 ms consumer
limit. Read `603F` for the latched fault. Restore heartbeat, explicitly reset
with zero target, clear reset bit 7, repeat the enable walk, and send a fresh
velocity to recover. Local acquisition faults require fixing the cause and
restarting the executable.

For an offline process smoke check without a configured SocketCAN interface:

```bash
PYTHONPATH=src/robocell_sim python -m robocell_sim.motors \
  --model src/robocell_sim/models/waffle_pi/scene.xml \
  --interface virtual --channel isolated-smoke --duration 1
```

The python-can virtual backend shares traffic only within one process;
separate host and emulator processes require SocketCAN `vcan0`. Unit and
integration tests allocate unique in-process virtual channels and use
independent bus endpoints rather than loopback commands or mocked sockets.

Run emulator tests and the repository's strict gate in the configured
ROS development environment:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=src/robocell_sim:$PYTHONPATH python -m pytest \
  src/robocell_sim/test/test_controlled.py \
  src/robocell_sim/test/test_actuated.py \
  src/robocell_sim/test/test_described.py \
  src/robocell_sim/test/test_motor_emulated.py --eo -q
```

A separate `canopen.RemoteNode` client was exercised against both emulator
endpoints and the actual Waffle Pi scene without ROS. It read profile revision
1, selected mode 3 over SDO, verified all three enable transitions, commanded
opposite wheel velocities, and read negative left and positive right encoder
positions and measured velocities. Stopping command renewal with heartbeat
still active produced `FF01` on both drives and zero target velocities.
This validates library-client interoperability and the physical boundary,
not complete device conformance or electrical CAN behavior. SocketCAN and
the GUI require a configured host and are not covered by that virtual-bus run.

The regular workshop launch runs these endpoints in its own MuJoCo world.
`motors.py` remains a separate world for standalone clients; it does not
control a different workshop process's model.

## Standalone host driver

`motor_driver.py` is the ROS executable for the standalone `motors.py` world.
Its host protocol lives under `robot/implementations/motor` and imports neither
MuJoCo nor ROS. `Hosted.connection` also exposes that host as a non-ROS
context; its `Pilot` accepts body commands with their original monotonic
arrival time, services the bus and returns paired joint measurements.

The ROS node is `motor_driver`. It subscribes to `/cmd_vel` as
`geometry_msgs/Twist` with depth one and publishes `/joint_states` as
`sensor_msgs/JointState`. Feedback contains encoder angles, measured wheel
velocities and the CAN acquisition timestamp, rather than ROS reception time.
It services CAN every 5 ms of wall time, sends host heartbeat every 100 ms
and renews enabled PDO targets every 20 ms. Startup verifies profile revision,
mode, encoder units, all fixed PDO mappings and all supervision values through
expedited SDO before walking the zero-target enable sequence. Both drives
finish configuration before one broadcast NMT-start, so their asynchronous
PDO acquisition periods begin together. The configured
mode is written and read back; the default supervision values must match.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `interface` | `socketcan` | python-can backend |
| `channel` | `vcan0` | Shared CAN interface |
| `left_node`, `right_node` | `1`, `2` | Distinct wheel IDs in 1..126 |
| `wheel_radius` | `0.033` | Wheel radius in meters |
| `wheel_separation` | `0.288` | Wheel track in meters |
| `wheel_limit` | `7.88` | Maximum wheel speed in rad/s, at most 7.88 |
| `physics_step` | `0.002` | Maximum paired acquisition skew in seconds |
| `left_joint`, `right_joint` | `wheel_left_joint`, `wheel_right_joint` | JointState names |
| `collision_guard` | `false` | Require fresh `/scan` and apply the existing 0.3 m by 0.2 m corridor guard |

Geometry values must be finite and positive. Node identities and geometry
are checked before opening CAN. The physics-step parameter must match the
selected model. Source commands must arrive after successful enable, and
must continue at more than 4 Hz to stay within the 250 ms command timeout.
Stopping the command source sends zero targets to both nodes and keeps the
host enabled. A fresh command after input expiry resumes motion immediately.
Transport or feedback failures instead quick-stop both nodes and leave the
host stopped; restored traffic cannot replay a cached velocity. Such failures
require explicit retry or recovery, followed by a command received after
enable completes. Producer and host heartbeat continue while a connected
host is stopped.

The driver uses `/clock` when it is available to supervise feedback gaps in
simulated time. The non-ROS standalone emulator does not publish `/clock`,
so without that topic the driver infers progress from the latest complete
acquisition of either wheel and also requires feedback within 300 ms of
wall time. This conservatively stops on a long pause when no external clock
is available; request a verified retry after resuming. A frozen external
`/clock` suspends only the simulated-time feedback alarm. Heartbeat and
application-command supervision always remain on monotonic wall time.

Each startup or retry has a five-second wall-time budget, including CAN-open
retries with 100 ms backoff and discovery of devices that start after the host.
Heartbeat discovery uses that overall budget; each expedited SDO request gets
at most two retries.
Once an attempt fails, the node stays alive for an explicit operator request:

```console
ros2 service call /motor_driver/retry std_srvs/srv/Trigger '{}'
ros2 service call /motor_driver/recover std_srvs/srv/Trigger '{}'
```

`retry` opens a new verified session without clearing faults. `recover`
explicitly authorizes zero-target fault reset before configuration and enable.
Fix the fault cause first; a local `FF02` still requires emulator restart.
The services acknowledge acceptance, and the log and transient-local
`/motor_driver/state` report enable or failure.
Both requests discard earlier commands and partial feedback, preserve already
observed encoder travel and establish fresh position baselines. Shutdown
sends zero/quick-stop and disable voltage to both nodes before closing CAN;
it does not wait indefinitely for an acknowledgement.

For an installed package, use `ros2 run robocell_sim motor_driver.py`. From
the source tree with ROS sourced, the equivalent command is:

```console
PYTHONPATH=src/robocell_sim:$PYTHONPATH python -m robocell_sim.motor_driver \
  --ros-args -p channel:=vcan0 -p left_node:=1 -p right_node:=2
```

Use a shared SocketCAN interface for separate processes. The `virtual` backend
is for isolated same-process tests. Run one motor driver per shared CAN bus.
The standalone world accepts body commands directly; the normal workshop
uses the guarded integration below.

Host tests cover independent node startup, fresh commands, forward/reverse
and turning, measured CAN feedback, command expiry at both physical actuators,
non-operational heartbeat, feedback loss with an advancing clock, nonfinite
input, pause/resume, explicit watchdog-fault recovery, PDO assembly and
expiry, pairing skew, signed encoder rollover and baseline discontinuities.
A same-process smoke check also ran the actual ROS entry point against the
Waffle Pi scene on an isolated virtual CAN channel: motion reached both
actuators, joint messages arrived through DDS, the ROS graph had one joint
publisher, and stopping command publication zeroed both actuator targets.
This is not an electrical-CAN or separate-process SocketCAN validation.


## Workshop integration

`workshop.launch.xml` starts the integrated emulators in `main.py` and one
`motor_driver.py` with `collision_guard=true`. `can_interface`, `can_channel`,
`left_node` and `right_node` are shared launch arguments. Defaults remain
SocketCAN `vcan0`, left 1 and right 2. The simulator has no direct velocity
command subscription, actuator assignment or joint-feedback publisher.
Its optional route windows and encoder log consume the CAN-derived
`/joint_states` instead of reading wheel sensors directly.

All command sources enter one driver-owned `Filtered` control. `/cmd_vel`
uses its monotonic ROS arrival time; `/keyboard/cmd_vel` is a
`geometry_msgs/TwistStamped` stamped at the original key event with system
wall time. The driver maps that age into monotonic time once. The keyboard
publisher emits only actual arrow/Enter events, and a delayed older source
cannot supersede newer input. Periodic guard reevaluation preserves the
original input time, so a fresh scan cannot renew an abandoned command.
Hold an arrow for key repeat or send a continuous ROS command stream.

The existing corridor geometry remains 0.3 m front/rear by 0.2 m half-width.
The ROS scan adapter bins the nearest valid return at each degree, accounting
for `angle_min` and either sign of `angle_increment`. An obstacle blocks the
relevant translation while leaving turning available. Before a scan arrives,
with a backwards scan time, or more than 500 ms after its acquisition while
simulation time advances, all motion is inhibited.

Drive service brackets every physics step, including each individual catch-up
step. A pause interrupts catch-up and continues protective CAN service on wall
time. The simulator's `/sim/pause` service uses `std_srvs/SetBool`; space in
the viewer toggles the same pause event. On exit, emulator contexts zero their
actuators before releasing the viewer and transport resources. Normal launch
runs odometry, IMU, lidar, robot-state publication and EKF on their existing
ROS paths, with one `/joint_states` publisher named `motor_driver`.

The repeatable headless recording, measured results, CAN traffic and compressed
ROS bag are under [docs/canopen](../canopen/README.md). The run verifies
keyboard guarding against a wall, stale commands, pause/resume, command-watchdog
faults, explicit fault recovery, drive restart, fresh-input requirements and
clean host shutdown through the actual workshop launch.
