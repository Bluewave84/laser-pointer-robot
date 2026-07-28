# laser-pointer-robot

ESP32 (also compatible boards are RPi Pico, RPi Pico 2, Arduino) firmware based on PlatformIO for a two-axis stepper-driven laser pointer robot.

## Build

Use PlatformIO with the `um_feathers2` environment:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

The firmware depends on these PlatformIO libraries from [platformio.ini](platformio.ini):

- `gin66/FastAccelStepper`
- `teemuatlut/TMCStepper`

## Current Firmware Shape

The current firmware is split into a small set of modules:

| File | Responsibility |
| --- | --- |
| [src/main.cpp](src/main.cpp) | Physical configuration, module wiring, serial command dispatch, range test, and pattern test orchestration. |
| [src/MotorAdapter.h](src/MotorAdapter.h) / [src/MotorAdapter.cpp](src/MotorAdapter.cpp) | Per-axis Motor adapter around FastAccelStepper and TMC2209 setup/runtime calls. |
| [src/StallDetector.h](src/StallDetector.h) / [src/StallDetector.cpp](src/StallDetector.cpp) | Stall confirmation by sampling UART-read `SG_RESULT` against the configured StallGuard threshold. |
| [src/HomingStateMachine.h](src/HomingStateMachine.h) / [src/HomingStateMachine.cpp](src/HomingStateMachine.cpp) | Homing Robot State transitions for X then Y, including arming, seeking, backoff, configured-range handling, and fault state. |

`MotorAdapter` preserves the ADR-0001 FastAccelStepper decision by wrapping the library instead of replacing it. `Axis` now carries only axis metadata and range state; hardware driver pointers and pins live behind the Motor adapter.

## Homing Behavior

Homing is sensorless StallGuard homing using TMC2209 UART reads. The firmware does not use a DIAG pin for stall detection.

For each axis, homing:

1. Arms by moving away from the expected zero end.
2. Seeks toward zero while `StallDetector` samples `SG_RESULT` after a settle distance.
3. Accepts a stall when `SG_RESULT <= SGTHRS` for the configured confirmation count.
4. Sets the zero position, backs off, and either uses the configured range or seeks the max end.
5. Repeats the same process for the Y axis.

Configured full-step ranges are set in [src/main.cpp](src/main.cpp):

```cpp
constexpr uint32_t X_AXIS_RANGE_FULLSTEP = 512;
constexpr uint32_t Y_AXIS_RANGE_FULLSTEP = 704;
```

Set either value to `0` to make that axis run the full FindZero + FindMax range discovery path.

## Serial Commands

Connect over USB serial at `115200` baud. Commands are single characters:

| Command | Description |
| --- | --- |
| `s` | Start homing X then Y. |
| `c` | Run the axis range sweep test after both ranges are known. |
| `1` | Run the square pattern test. |
| `2` | Run the diamond pattern test. |
| `3` | Run the figure-8 pattern test. |
| `4` | Run the spiral pattern test. |
| `d` | Print one StallGuard diagnostic sample. |
| `p` | Print pattern command help. |
| `x` | Abort active homing, range test, or pattern test. |

Pattern tests require successful homing first so both axis ranges are known.
