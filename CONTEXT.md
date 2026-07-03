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
A requested axis angle that the robot should move toward.
_Avoid_: Channel value, motor step position

**Motor**:
A stepper actuator that moves an axis and is measured by step position.
_Avoid_: Axis, channel

**Command**:
A domain intent sent to the laser pointer robot, such as setting an axis target, setting motion limits, zeroing the current position, performing an emergency stop, or sending hello/keep-alive.
_Avoid_: Message frame, UDP packet, serial input

**Robot State**:
The durable operating truth of the robot after commands are consumed, including axis targets, motion limits, emergency-stop state, and current position.
_Avoid_: Current command, mode

**Transport**:
The delivery mechanism that carries commands to the robot.
_Avoid_: Command

**Message Frame**:
The byte-level representation of a command on a transport.
_Avoid_: Command

**Command Input**:
The responsibility that parses transports and message frames into commands.
_Avoid_: Network, robot motion

**Robot Motion**:
The responsibility that consumes commands and updates robot state, axis targets, motor positions, timers, and driver enablement.
_Avoid_: Command input, robot configuration

**Robot Configuration**:
The responsibility that provides physical configuration and default motion limits.
_Avoid_: Runtime state, command input

**Physical Configuration**:
The fixed hardware facts of a laser pointer robot build, such as pins, motor geometry, axis limits, and initial position.
_Avoid_: Motion limits, command settings

**Motion Limits**:
The adjustable operating constraints used while moving, such as speed and acceleration limits.
_Avoid_: Physical configuration, hardware facts

**Emergency Stop**:
An overriding safety state that disables movement and prevents future movement until the robot reboots.
_Avoid_: Mode, pause, keep-alive, zero current position

**Zero Current Position**:
A command that declares the robot's current pose to be zero without discovering a physical reference.
_Avoid_: Calibration, homing

**Stop Active Movement**:
A command effect that stops the robot's current movement without changing position, motion limits, or emergency-stop state.
_Avoid_: Hello, keep-alive, emergency stop
