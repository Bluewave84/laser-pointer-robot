# Migrate motor control from manual ESP32 timers to FastAccelStepper library

We replaced custom ISR-driven stepper control (using `esp_timer` callbacks at 31kHz) with the FastAccelStepper library, which runs from the main loop via `engine->nextTick()` every iteration. This shift eliminates manual acceleration/deceleration logic, simplifies the control model, and leverages a battle-tested motion profile algorithm optimized for ESP32's GPIO performance.

## Status

Accepted

## Considered Options

- **AccelStepper**: Mature, broad community, but uses slower `digitalWrite()` on ESP32
- **Manual ESP32 timers** (current): Full control, but high complexity for overshoot compensation and multi-axis coordination
- **FastAccelStepper**: ESP32-optimized GPIO, smaller community, but acceptable risk for this application

We chose FastAccelStepper because it balances performance (fast GPIO writes for 16kHz+ stepping) with maintainability (library-owned motion state machine eliminates custom acceleration logic).

## Consequences

**Simplifications:**
- Eliminates ~150 lines of ISR and timer management code
- Removes manual position, speed, and acceleration state variables
- Command handlers now directly call `stepper->moveTo()` and library update functions

**Changes to motion behavior:**
- Built-in S-curve deceleration replaces custom overshoot compensation (look-ahead prevents overshooting)
- Acceleration profile is library-controlled (cannot be customized at runtime, only via `setMaxSpeed()/setAcceleration()`)
- Motion state is library-owned; commands are no longer shadowed by a separate control loop

**I/O assumptions:**
- Main loop calls `engine->nextTick()` without throttling; library manages timing via `micros()`
- Non-blocking command input is required (already satisfied by WiFi UDP and serial polling)

**Integration points:**
- Driver enable pin remains manually managed (shared CNC_ENABLE_PIN for both motors)
- Direction pin inversion retained via compile-time `#ifdef INVERT_M*_AXIS`
- Emergency stop uses library's `forceStop()` with a local flag to prevent post-estop commands
