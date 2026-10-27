# CAN protocol of the IMU board

This document is the contract between the IMU board and the host that reads
it. The board emulator and the IMU driver are both written against it alone,
the way a driver is written against a sensor's datasheet without seeing its
firmware, and the example frames below are the reference bytes of their tests.

## Overview

The board sends its readings on its own, with no request from the host.

Each reading holds the angular velocity and the linear acceleration in SI
units, with the noise and the bias of the sensor left in. The board sends no
orientation: the host estimates the heading, since only the host can fuse the
gyro with the wheels and the lidar.

The bus is classic CAN with 11-bit identifiers and at most 8 data bytes per
frame.

## Frames

| Identifier            | Bytes 0–1 | Bytes 2–3 | Bytes 4–5 | Byte 6             | Byte 7           |
| --------------------- | --------- | --------- | --------- | ------------------ | ---------------- |
| `0x101` gyro          | x         | y         | z         | measurement number | `0x00`, reserved |
| `0x102` accelerometer | x         | y         | z         | measurement number | `0x00`, reserved |

Every frame carries all 8 data bytes.

- x, y and z are `int16`, little-endian, in two's complement.
- The gyro counts 0.001 rad/s per unit, so its range is ±32.767 rad/s.
- The accelerometer counts 0.001 m/s² per unit, so its range is ±32.767 m/s².
- A value past the range is clipped to `-32768` or `32767`.
- The measurement number is an unsigned byte.
- The reserved byte is sent as `0x00`, and the receiver ignores it.

A measurement takes two frames because six `int16` and a number take 13 bytes,
more than one classic frame holds.

## Axes and signs

Every vector is in the `imu_link` frame: x forward, y left and z up, as in
REP 103.

A counterclockwise turn seen from above gives a positive gyro z.

The accelerometer includes gravity: at rest on a level floor it reads about
`+9.81` on z.

## Timing

The board takes one measurement every 10 ms, 100 per second.

Each measurement goes out as two frames back to back, `0x101` first and
`0x102` second. Both frames carry the same measurement number. The number
grows by one per measurement and wraps from 255 to 0.

The frames carry no time of their own. The receiver stamps a measurement when
it arrives, so the stamp jitters by up to the receiver's polling period. To
integrate the gyro exactly, the receiver can count 10 ms per measurement
number instead.

## Receiving

The receiver pairs the two frames of a measurement by their number.

A gyro frame that never gets its accelerometer frame loses its measurement,
and so does an accelerometer frame whose number does not match the waiting
gyro frame. The receiver counts each such loss.

The number of a measurement follows the previous one by +1, or wraps from 255
to 0. Any other jump means that whole measurements were lost, and the
receiver counts them as the jump minus one, modulo 256.

## Bus

The bit rate is 500 kbit/s. A classic frame with 8 data bytes takes about 135
bits at worst, with bit stuffing and the gap between frames, so 200 frames per
second load the bus to about 5 %. The virtual bus `vcan0` ignores the bit
rate, but a real bus would not.

The IMU shares the bus with the CANopen motor drives of later work. In the
CANopen predefined connection set `0x100` is TIME and the device PDOs start at
`0x181`, so `0x101` and `0x102` collide with nothing. A lower identifier wins
arbitration, so the IMU frames outrank the drive PDOs but yield to NMT, SYNC
and EMCY.

## Example frames

Measurement number 17, gyro (0, 0, 0.5) rad/s and accelerometer
(0, 0, 9.81) m/s²:

```
0x101  00 00 00 00 F4 01 11 00
0x102  00 00 00 00 52 26 11 00
```

- `F4 01` is `0x01F4`, that is 500 units, or 0.5 rad/s.
- `52 26` is `0x2652`, that is 9810 units, or 9.81 m/s².
- `11` is 17, the measurement number.

A negative value: −0.3 rad/s is −300 units, `0xFED4` in two's complement,
sent as `D4 FE`.

## Out of scope

- Commands from the host to the board.
- CAN FD.
- Orientation computed on the board.
