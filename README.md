# laser-pointer-robot

Firmware for a two-axis stepper-driven laser pointer robot on an Unexpected Maker FeatherS2. The active PlatformIO sketch is [src/main.cpp](src/main.cpp); the older Arduino sketch remains under [src/Laser_Pointer_Robot](src/Laser_Pointer_Robot) for now.

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
| [src/main.cpp](src/main.cpp) | Physical configuration, module wiring, and firmware setup/loop coordination. |
| [src/CommandSystem.h](src/CommandSystem.h) / [src/CommandSystem.cpp](src/CommandSystem.cpp) | Catalog-backed Command Input and dispatch: command groups, commands, parameters, Serial aliases, Web names, help text, and Robot Motion routing. |
| [src/MotorAdapter.h](src/MotorAdapter.h) / [src/MotorAdapter.cpp](src/MotorAdapter.cpp) | Per-axis Motor adapter around FastAccelStepper and TMC2209 setup/runtime calls. |
| [src/StallDetector.h](src/StallDetector.h) / [src/StallDetector.cpp](src/StallDetector.cpp) | Stall confirmation by sampling UART-read `SG_RESULT` against the configured StallGuard threshold. |
| [src/HomingStateMachine.h](src/HomingStateMachine.h) / [src/HomingStateMachine.cpp](src/HomingStateMachine.cpp) | Homing Robot State transitions for X then Y, including arming, seeking, backoff, configured-range handling, and fault state. |
| [src/TestController.h](src/TestController.h) / [src/TestController.cpp](src/TestController.cpp) | Range-test and pattern-test orchestration, including test state, waypoints, motion profiles, updates, and cancellation. |

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

## Commands

Commands are defined as command groups, commands, and optional parameters in the static command catalog. USB serial is the active Transport today; Web names are included in the catalog so a future Web API can produce the same Commands without reimplementing Robot Motion dispatch.

Parameterized position commands use normalized axis positions from `0` to `10000`, where `0` is the known minimum usable position and `10000` is the known maximum usable position after homing. The dispatcher converts normalized positions to current microstep positions using the known axis ranges, so commands do not depend on the configured microstepping resolution.

Connect over USB serial at `115200` baud. Most Serial aliases are single characters; parameterized movement uses a newline-terminated line command.

| Group | Command | Parameter | Serial | Web name | Description |
| --- | --- | --- | --- | --- | --- |
| Homing | StartHoming | none | `s` | `homing.start` | Start homing X then Y. |
| RobotMotion | AbortActive | none | `x` | `motion.abort` | Abort active homing, range test, or pattern test. |
| RobotMotion | MoveToPosition | `x=0..10000, y=0..10000` | `m x y` | `motion.position` | Move both axes to a normalized position after homing. |
| StallDetection | PrintStallGuardSample | none | `d` | `stall.sample` | Print one StallGuard diagnostic sample. |
| TestController | StartRangeTest | none | `c` | `test.range` | Run the axis range sweep test after both ranges are known. |
| TestController | StartPatternTest | `Pattern=Square` | `1` | `test.pattern.square` | Run the square pattern test. |
| TestController | StartPatternTest | `Pattern=Diamond` | `2` | `test.pattern.diamond` | Run the diamond pattern test. |
| TestController | StartPatternTest | `Pattern=Figure8` | `3` | `test.pattern.figure8` | Run the figure-8 pattern test. |
| TestController | StartPatternTest | `Pattern=Spiral` | `4` | `test.pattern.spiral` | Run the spiral pattern test. |
| Catalog | PrintCommandCatalog | none | `p` | `catalog.print` | Print the command catalog. |

For example, send `m 5000 5000` followed by Enter to move both axes to the center. Pattern and position commands require successful homing first so both axis ranges are known.
