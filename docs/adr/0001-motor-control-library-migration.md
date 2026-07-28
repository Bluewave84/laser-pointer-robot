# Migrate motor control from manual ESP32 timers to FastAccelStepper library

We replaced custom ISR-driven stepper control (using `esp_timer` callbacks at 31kHz) with the FastAccelStepper library. This shift eliminates manual acceleration/deceleration logic, simplifies the control model, and leverages a battle-tested motion profile algorithm optimized for ESP32's GPIO performance.

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
- Robot Motion now issues movement through a per-axis Motor adapter that wraps FastAccelStepper runtime calls

**Changes to motion behavior:**
- Built-in S-curve deceleration replaces custom overshoot compensation (look-ahead prevents overshooting)
- Acceleration profile is library-controlled (cannot be customized at runtime, only via `setSpeedInHz()/setAcceleration()`)
- Motion state is library-owned; commands are no longer shadowed by a separate control loop

**I/O assumptions:**
- Motion execution is interrupt/task-driven by FastAccelStepper once moves are queued
- Non-blocking command input is required; the active firmware satisfies this with USB serial polling

**Integration points:**
- Driver enable pin remains configured through FastAccelStepper and is shared by both motors
- Direction pin setup is owned by the Motor adapter from immutable construction-time pin facts
- Stop/abort paths use the Motor adapter's `forceStop()` operation, which delegates to FastAccelStepper
- Sensorless homing uses TMC2209 UART `SG_RESULT` sampling through the Motor adapter; DIAG pin interrupts are not part of the current homing design
