# Laser Pointer Robot

This context describes the domain language for firmware that aims a laser using a two-axis stepper-driven platform.

## Language

**Laser Pointer Robot**:
A two-axis platform that aims a laser by moving motors to requested axis angles.
_Avoid_: Motion controller, CNC machine

**Axis**:
A user-facing aiming dimension measured in degrees.
_Avoid_: Channel, motor, stepper

**Axis Target**:
A requested axis position that the robot should move toward, expressed either as a user-facing angle or as a normalized range position.
_Avoid_: Channel value, raw microstep position

**Normalized Axis Position**:
A transport-safe axis target from `0` to `10000`, where `0` is the known minimum usable position and `10000` is the known maximum usable position after homing.
_Avoid_: Microstep count, full-step count

**Motor**:
A stepper actuator that moves an axis and is measured by step position.
_Avoid_: Axis, channel

**Motor Adapter**:
The responsibility that hides per-axis motor driver and stepper-library details behind movement, position, profile, enablement, and StallGuard sampling operations.
_Avoid_: Axis, robot state

**Command**:
A domain intent sent to the laser pointer robot, such as homing, moving to an axis target, running a range test, running a pattern test, printing diagnostics, or aborting active motion.
_Avoid_: Serial character, message frame, transport input

**Robot State**:
The durable operating truth of the robot after commands are consumed, including active homing phase, known axis ranges, test activity, fault state, and current motor position.
_Avoid_: Current command, mode

**Transport**:
The delivery mechanism that carries commands to the robot.
_Avoid_: Command

**Command Input**:
The responsibility that parses transport input, currently single-character USB serial input, into catalog-backed Commands.
_Avoid_: Network, robot motion

**Command Catalog**:
The responsibility that defines command groups, commands, parameters, transport aliases, Web names, and help text in one static table.
_Avoid_: Transport parser, robot motion

**Command Dispatcher**:
The responsibility that consumes Commands and invokes Robot Motion modules without knowing which Transport produced the command.
_Avoid_: Serial command parser, Web API handler

**Robot Motion**:
The responsibility that consumes commands and updates robot state, axis targets, motor positions, timers, and driver enablement.
_Avoid_: Command input, robot configuration

**Test Controller**:
The Robot Motion responsibility that owns range-test and pattern-test orchestration, including test state, waypoints, motion profiles, update steps, and cancellation.
_Avoid_: Homing state, command input

**Homing**:
The Robot Motion responsibility that discovers a physical reference and usable axis range by moving an axis until StallGuard reports a stall.
_Avoid_: Zero current position, range test

**Stall Detection**:
The responsibility that decides whether a motor has stalled by comparing UART-read `SG_RESULT` values to the configured StallGuard threshold after a settle distance.
_Avoid_: DIAG pin interrupt, homing state

**Robot Configuration**:
The responsibility that provides physical configuration and default motion limits.
_Avoid_: Runtime state, command input

**Physical Configuration**:
The fixed hardware facts of a laser pointer robot build, such as pins, motor geometry, axis limits, and initial position.
_Avoid_: Motion limits, command settings

**Motion Limits**:
The adjustable operating constraints used while moving, such as speed and acceleration limits.
_Avoid_: Physical configuration, hardware facts

**Stop Active Movement**:
A command effect that stops the robot's current movement without changing position, motion limits, or emergency-stop state.
_Avoid_: Hello, keep-alive, emergency stop
