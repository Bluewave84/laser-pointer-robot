# laser-pointer-robot

Firmware for a two-axis stepper-driven laser pointer robot on Feather-format ESP32 boards. The active PlatformIO sketch is [src/main.cpp](src/main.cpp); the older Arduino sketch remains under [src/Laser_Pointer_Robot](src/Laser_Pointer_Robot) for now.

Supported PlatformIO targets:

- `um_feathers2`: Unexpected Maker FeatherS2.
- `adafruit_feather_esp32_v2`: Adafruit ESP32 Feather V2. Pin capabilities and the firmware pin map are documented in [docs/hardware/adafruit-esp32-feather-v2.md](docs/hardware/adafruit-esp32-feather-v2.md).

## Build

Use PlatformIO with the target board environment:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run --environment um_feathers2
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run --environment adafruit_feather_esp32_v2
```

The firmware depends on these PlatformIO libraries from [platformio.ini](platformio.ini):

- `gin66/FastAccelStepper`
- `teemuatlut/TMCStepper`
- `bblanchon/ArduinoJson`

Create an untracked Wi-Fi credentials header before building the Web API firmware:

```powershell
Copy-Item src\secrets.example.h src\secrets.h
```

Then edit `src/secrets.h` locally:

```cpp
constexpr const char *WIFI_SSID = "your-ssid";
constexpr const char *WIFI_PASSWORD = "your-password";
```

## Current Firmware Shape

The current firmware is split into a small set of modules:

| File | Responsibility |
| --- | --- |
| [src/main.cpp](src/main.cpp) | Physical configuration, module wiring, and firmware setup/loop coordination. |
| [src/BoardConfiguration.h](src/BoardConfiguration.h) | Compile-time board selection for pin maps and FastAccelStepper CPU affinity. |
| [src/CommandSystem.h](src/CommandSystem.h) / [src/CommandSystem.cpp](src/CommandSystem.cpp) | Catalog-backed Command Input and dispatch: command groups, commands, parameters, Serial aliases, Web names, help text, and Robot Motion routing. |
| [src/WebCommandInput.h](src/WebCommandInput.h) / [src/WebCommandInput.cpp](src/WebCommandInput.cpp) | HTTP Web Command Input transport that exposes the Command Catalog Web names as JSON API endpoints. |
| [src/MotorAdapter.h](src/MotorAdapter.h) / [src/MotorAdapter.cpp](src/MotorAdapter.cpp) | Per-axis Motor adapter around FastAccelStepper and TMC2209 setup/runtime calls. |
| [src/StallDetector.h](src/StallDetector.h) / [src/StallDetector.cpp](src/StallDetector.cpp) | Stall confirmation by sampling UART-read `SG_RESULT` against the configured StallGuard threshold. |
| [src/HomingStateMachine.h](src/HomingStateMachine.h) / [src/HomingStateMachine.cpp](src/HomingStateMachine.cpp) | Homing Robot State transitions for X then Y, including arming, seeking, backoff, configured-range handling, and fault state. |
| [src/TestController.h](src/TestController.h) / [src/TestController.cpp](src/TestController.cpp) | Range-test and pattern-test orchestration, including test state, waypoints, motion profiles, updates, and cancellation. |

`BoardConfiguration` selects the board-specific pins through PlatformIO `build_flags`. For the Adafruit ESP32 Feather V2 target, FastAccelStepper is initialized on CPU core `0`, keeping its stepping task off the Arduino loop core on the dual-core ESP32. `MotorAdapter` preserves the ADR-0001 FastAccelStepper decision by wrapping the library instead of replacing it. `Axis` now carries only axis metadata and range state; hardware driver pointers and pins live behind the Motor adapter.

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

Commands are defined as command groups, commands, and optional parameters in the static command catalog. USB serial and the HTTP Web API are transports over the same catalog-backed Commands, so they share Robot Motion dispatch behavior.

Parameterized position commands use normalized axis positions from `0` to `10000`, where `0` is the known minimum usable position and `10000` is the known maximum usable position after homing. The dispatcher converts normalized positions to current microstep positions using the known axis ranges, so commands do not depend on the configured microstepping resolution.

Connect over USB serial at `115200` baud. Most Serial aliases are single characters; parameterized movement and runtime setting commands use newline-terminated line commands.

| Group | Command | Parameter | Serial | Web name | Description |
| --- | --- | --- | --- | --- | --- |
| Homing | StartHoming | none | `s` | `homing.start` | Start homing X then Y. |
| RobotMotion | AbortActive | none | `x` | `motion.abort` | Abort active homing, range test, or pattern test. |
| RobotMotion | MoveToPosition | `x=0..10000, y=0..10000` | `m x y` | `motion.position` | Move both axes to a normalized position after homing. |
| RobotMotion | MoveToZero | none | `z` | `motion.zero` | Move both axes to normalized position `0 0` after homing. |
| RobotMotion | SetMicrosteps | `n=1,2,4,8,16,32,64,128,256` | `u n` | `motion.microsteps` | Set runtime microstepping while idle, preserving normalized position. |
| RobotMotion | SetSpeed | `hz=1..50000` | `v hz` | `motion.speed` | Set runtime speed for position and pattern moves while idle. |
| RobotMotion | SetAcceleration | `n=1..100000` | `a n` | `motion.acceleration` | Set runtime acceleration for position and pattern moves while idle. |
| StallDetection | PrintStallGuardSample | none | `d` | `stall.sample` | Print one StallGuard diagnostic sample. |
| TestController | StartRangeTest | none | `c` | `test.range` | Run the axis range sweep test after both ranges are known. |
| TestController | StartPatternTest | `Pattern=Square` | `1` | `test.pattern.square` | Run the square pattern test. |
| TestController | StartPatternTest | `Pattern=Diamond` | `2` | `test.pattern.diamond` | Run the diamond pattern test. |
| TestController | StartPatternTest | `Pattern=Figure8` | `3` | `test.pattern.figure8` | Run the figure-8 pattern test. |
| TestController | StartPatternTest | `Pattern=Spiral` | `4` | `test.pattern.spiral` | Run the spiral pattern test. |
| Catalog | PrintCommandCatalog | none | `p` | `catalog.print` | Print the command catalog. |

For example, send `m 5000 5000` followed by Enter to move both axes to the center. Pattern and position commands require successful homing first so both axis ranges are known. Homing always uses its fixed homing microstepping and motion profile; runtime speed and acceleration apply to position and pattern moves, while the range test keeps its dedicated range-test profile.

## Web API

The firmware connects to Wi-Fi in station mode using `src/secrets.h`. When connected, it prints the assigned IP address over Serial and starts a JSON HTTP API on port `80`.

| Method | Path | Description |
| --- | --- | --- |
| `GET` | `/api/status` | Return Wi-Fi, homing, test, axis, and runtime motion status. |
| `GET` | `/api/commands` | Return the command catalog with Web names, Serial aliases, parameter kinds, and help text. |
| `POST` | `/api/commands/{webName}` | Dispatch a catalog command by Web name. |

Command requests use JSON bodies only. No-parameter commands may use an empty body or `{}`. Position commands use normalized axis positions:

```json
{ "x": 5000, "y": 5000 }
```

Runtime setting commands use `value`:

```json
{ "value": 12000 }
```

Command responses include the dispatcher status and message:

```json
{ "status": "accepted", "message": "Move command processed." }
```

HTTP status codes map to command results: `202` for accepted commands, `400` for invalid parameters or unknown commands, `409` for busy robot state, and `422` for rejected or faulted commands.
