// ESP32 CNC laser pointer firmware - REFACTORED VERSION
// Improvements: error handling, state management, bounds checking, frame-rate independence

#include <Arduino.h>
#include "Configuration.h"

#define VERSION "ESP32 Laser Pointer v2 (Refactored)"

// ============================================================================
// CONFIGURATION & CONSTANTS
// ============================================================================

// Control loop timing
#define DT_THRESHOLD_US 1000              // Control loop update period (microseconds)
#define COMMAND_TIMEOUT_MS 5000           // Max time to wait for command completion
#define WATCHDOG_TIMEOUT_MS 10000         // Watchdog: auto-stop if no update

// Input scaling
#define CH_SCALING_FACTOR 100.0f          // Channel input divisor
#define MAX_AXIS_CHANGE_PER_FRAME 45.0f   // Rate limit for safety (degrees/frame)

// Debug & diagnostics
#define ENABLE_DEBUG 1                    // Set to 1 for verbose debug output
#define DEBUG_INTERVAL_MS 1000            // Debug telemetry interval

#if ENABLE_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ============================================================================
// OPERATING MODES
// ============================================================================

enum ControlMode {
  MODE_IDLE = 0,
  MODE_JOY = 1,
  MODE_RESERVED_2 = 2,
  MODE_RESERVED_3 = 3,
  MODE_EMERGENCY_STOP = 4,
  MODE_CALIBRATION = 5
};

// ============================================================================
// STATE MANAGEMENT
// ============================================================================

typedef struct {
  bool initialized;
  bool network_ok;
  bool motors_ok;
  bool emergency_active;
} SystemStatus;

typedef struct {
  uint8_t current_mode;
  uint8_t pending_mode;
  uint32_t mode_change_time;
  bool mode_locked;  // Prevent mode changes during critical operations
} ControlState;

SystemStatus system_status = {false, false, false, false};
ControlState control_state = {MODE_IDLE, MODE_IDLE, 0, false};

// Timing variables
volatile uint32_t command_last_received_ms = 0;
uint32_t debug_last_print_ms = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialize all system hardware with error checking
 * @return true if all systems initialized successfully, false otherwise
 */
bool initializeSystem() {
  DEBUG_PRINTLN("[INIT] Starting system initialization...");
  
  // Initialize motion hardware
  if (!initializeMotionHardware()) {
    Serial.println("[ERROR] Motion hardware initialization failed!");
    system_status.motors_ok = false;
    return false;
  }
  system_status.motors_ok = true;
  DEBUG_PRINTLN("[INIT] Motion hardware OK");

  // Initialize network
  if (!NetworkBegin()) {
    Serial.println("[ERROR] Network initialization failed!");
    system_status.network_ok = false;
    return false;
  }
  system_status.network_ok = true;
  DEBUG_PRINTLN("[INIT] Network OK");

  system_status.initialized = true;
  return true;
}

/**
 * @brief Configure initial motor positions and speed/accel parameters
 */
void configureMotorDefaults() {
  position_M1 = (int32_t)(ROBOT_INITIAL_POSITION_M1 * M1_AXIS_STEPS_PER_UNIT);
  position_M2 = (int32_t)(ROBOT_INITIAL_POSITION_M2 * M2_AXIS_STEPS_PER_UNIT);
  target_position_M1 = position_M1;
  target_position_M2 = position_M2;

  configSpeed(MAX_SPEED_M1, MAX_SPEED_M2);
  configAcceleration(MAX_ACCEL_M1, MAX_ACCEL_M2);
  setSpeedAcc();

  DEBUG_PRINT("[MOTOR] Initial M1 position: ");
  DEBUG_PRINTLN(position_M1);
  DEBUG_PRINT("[MOTOR] Initial M2 position: ");
  DEBUG_PRINTLN(position_M2);
}

/**
 * @brief Initialize I/O pins and prepare for operation
 */
void setupPins() {
  // Motor control pins
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(CNC_ENABLE_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  // Set safe initial states
  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Y_STEP_PIN, LOW);
  digitalWrite(X_DIR_PIN, LOW);
  digitalWrite(Y_DIR_PIN, LOW);
  digitalWrite(CNC_ENABLE_PIN, HIGH);  // Disabled initially
  digitalWrite(STATUS_LED_PIN, LOW);
}

/**
 * @brief Print boot diagnostics to serial
 */
void printBootInfo() {
  Serial.println(VERSION);
  Serial.println("Booting ESP32 motion controller...");
  
  Serial.print("X STEP/DIR: ");
  Serial.print(X_STEP_PIN);
  Serial.print("/");
  Serial.println(X_DIR_PIN);
  
  Serial.print("Y STEP/DIR: ");
  Serial.print(Y_STEP_PIN);
  Serial.print("/");
  Serial.println(Y_DIR_PIN);
  
  Serial.print("CNC ENABLE: ");
  Serial.println(CNC_ENABLE_PIN);
  
  Serial.print("STATUS LED: ");
  Serial.println(STATUS_LED_PIN);
}

/**
 * @brief Main setup routine - called once at startup
 */
void setup() {
  delay(100);
  Serial.begin(115200);
  delay(500);

  printBootInfo();
  setupPins();

  // Initialize system with error checking
  if (!initializeSystem()) {
    Serial.println("[FATAL] System initialization failed. Halting.");
    digitalWrite(STATUS_LED_PIN, LOW);
    while (1) {
      delay(1000);  // Halt
    }
  }

  configureMotorDefaults();

  // Enable motors (CNC_ENABLE_PIN LOW = enabled)
  system_status.emergency_active = false;
  digitalWrite(CNC_ENABLE_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, HIGH);

  // Initialize timers
  timer_old = micros();
  slow_timer_old = millis();
  laser_timer_old = millis();
  command_last_received_ms = millis();

  Serial.println("[READY] System ready for commands.");
}

// ============================================================================
// INPUT PROCESSING & VALIDATION
// ============================================================================

/**
 * @brief Validate and constrain axis input to safe bounds
 * @param raw_input Raw channel value
 * @param min_limit Minimum allowed value
 * @param max_limit Maximum allowed value
 * @return Constrained axis value in degrees
 */
float validateAxisInput(int16_t raw_input, float min_limit, float max_limit) {
  if (raw_input == NODATA) {
    return 0.0f;
  }
  
  float scaled = raw_input / CH_SCALING_FACTOR;
  
  // Constrain to physical limits
  if (scaled < min_limit) {
    DEBUG_PRINT("[WARN] Axis clamped below minimum: ");
    DEBUG_PRINTLN(scaled);
    return min_limit;
  }
  if (scaled > max_limit) {
    DEBUG_PRINT("[WARN] Axis clamped above maximum: ");
    DEBUG_PRINTLN(scaled);
    return max_limit;
  }
  
  return scaled;
}

/**
 * @brief Process joystick/analog input commands with bounds checking
 */
void processJoystickInput() {
  if (control_state.mode_locked) {
    DEBUG_PRINTLN("[WARN] Mode locked, ignoring joystick input");
    return;
  }

  bool has_valid_input = false;
  float axis1 = 0.0f, axis2 = 0.0f;

  // Validate and scale inputs
  if (iCH1 != NODATA) {
    axis1 = validateAxisInput(iCH1, ROBOT_MIN_A1, ROBOT_MAX_A1);
    has_valid_input = true;
  }

  if (iCH2 != NODATA) {
    axis2 = validateAxisInput(iCH2, ROBOT_MIN_A2, ROBOT_MAX_A2);
    has_valid_input = true;
  }

  if (has_valid_input) {
    setAxis1(axis1);
    setAxis2(axis2);
    setSpeedAcc();
    command_last_received_ms = millis();
    DEBUG_PRINT("[JOY] Axis1: ");
    DEBUG_PRINT(axis1);
    DEBUG_PRINT(", Axis2: ");
    DEBUG_PRINTLN(axis2);
  }
}

/**
 * @brief Handle mode transitions with locking mechanism
 * @param new_mode The requested control mode
 */
void requestModeChange(uint8_t new_mode) {
  if (new_mode == control_state.current_mode) {
    return;  // Already in this mode
  }

  if (control_state.mode_locked) {
    DEBUG_PRINTLN("[WARN] Mode change rejected: operation in progress");
    return;
  }

  control_state.pending_mode = new_mode;
  control_state.mode_change_time = millis();
  DEBUG_PRINT("[MODE] Requesting mode change to: ");
  DEBUG_PRINTLN(new_mode);
}

// ============================================================================
// COMMAND HANDLERS
// ============================================================================

/**
 * @brief Handle emergency stop command
 */
void handleEmergencyStop() {
  if (!system_status.emergency_active) {
    DEBUG_PRINTLN("[EMERGENCY] STOP activated!");
    system_status.emergency_active = true;
    control_state.mode_locked = true;
    emergencyStop();
    digitalWrite(STATUS_LED_PIN, LOW);
    command_last_received_ms = millis();
  }
}

/**
 * @brief Handle motor calibration command
 */
void handleCalibration() {
  if (control_state.mode_locked) {
    DEBUG_PRINTLN("[CALIB] Calibration already in progress");
    return;
  }

  DEBUG_PRINTLN("[CALIB] Starting motor calibration...");
  control_state.mode_locked = true;
  
  motorsCalibration();
  setSpeedAcc();
  
  control_state.mode_locked = false;
  DEBUG_PRINTLN("[CALIB] Calibration complete");
  command_last_received_ms = millis();
}

/**
 * @brief Resume normal operation after emergency stop
 */
void resumeFromEmergencyStop() {
  if (system_status.emergency_active) {
    DEBUG_PRINTLN("[RESUME] Clearing emergency stop");
    system_status.emergency_active = false;
    control_state.mode_locked = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
    command_last_received_ms = millis();
  }
}

// ============================================================================
// MOTION CONTROL & WATCHDOG
// ============================================================================

/**
 * @brief Check for command timeout and trigger safety stop if needed
 */
void checkCommandTimeout() {
  uint32_t time_since_command = millis() - command_last_received_ms;
  
  if (time_since_command > WATCHDOG_TIMEOUT_MS && 
      control_state.current_mode != MODE_IDLE && 
      !system_status.emergency_active) {
    
    Serial.print("[WATCHDOG] No command for ");
    Serial.print(time_since_command);
    Serial.println("ms - triggering safety stop");
    
    handleEmergencyStop();
  }
}

/**
 * @brief Update position control at fixed timestep
 * Receives actual dt instead of hardcoding, enabling frame-rate independence
 */
void updateMotionControl() {
  timer_value = micros();
  dt = (int)(timer_value - timer_old);
  
  if (dt >= DT_THRESHOLD_US) {
    timer_old = timer_value;
    
    // Pass actual dt for frame-rate independent motion
    positionControl(dt);
    
    DEBUG_PRINT("[MOTION] dt=");
    DEBUG_PRINT(dt);
    DEBUG_PRINT("us, M1_pos=");
    DEBUG_PRINT(position_M1);
    DEBUG_PRINT(", M2_pos=");
    DEBUG_PRINTLN(position_M2);
  }
}

/**
 * @brief Print periodic debug telemetry
 */
void printDebugTelemetry() {
  if (!ENABLE_DEBUG) return;
  
  uint32_t now = millis();
  if (now - debug_last_print_ms < DEBUG_INTERVAL_MS) {
    return;
  }
  debug_last_print_ms = now;

  Serial.print("[TELEMETRY] Mode=");
  Serial.print(control_state.current_mode);
  Serial.print(" | M1=");
  Serial.print(position_M1);
  Serial.print(" | M2=");
  Serial.print(position_M2);
  Serial.print(" | Target_M1=");
  Serial.print(target_position_M1);
  Serial.print(" | Target_M2=");
  Serial.print(target_position_M2);
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

/**
 * @brief Main control loop
 */
void loop() {
  // Read incoming commands
  MsgRead();
  USBMsgRead();

  // Process any pending mode changes
  if (newMessage) {
    newMessage = 0;

    // Handle mode-based command routing
    switch (control_state.pending_mode) {
      case MODE_JOY:
        processJoystickInput();
        control_state.current_mode = MODE_JOY;
        break;

      case MODE_EMERGENCY_STOP:
        handleEmergencyStop();
        control_state.current_mode = MODE_EMERGENCY_STOP;
        break;

      case MODE_CALIBRATION:
        handleCalibration();
        control_state.current_mode = MODE_CALIBRATION;
        break;

      case MODE_IDLE:
        resumeFromEmergencyStop();
        control_state.current_mode = MODE_IDLE;
        break;

      default:
        DEBUG_PRINT("[WARN] Unknown mode: ");
        DEBUG_PRINTLN(control_state.pending_mode);
        break;
    }

    control_state.pending_mode = control_state.current_mode;
  }

  // Update motion at fixed timestep
  updateMotionControl();

  // Safety checks
  checkCommandTimeout();

  // Periodic diagnostics
  printDebugTelemetry();
}
