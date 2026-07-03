// ESP32 motion control for the laser pointer platform.

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>

static esp_timer_handle_t motor1_timer = nullptr;
static esp_timer_handle_t motor2_timer = nullptr;
static bool motor1_timer_running = false;
static bool motor2_timer_running = false;

static uint32_t speedToPeriodUs(int32_t speed) {
  if (speed == 0) {
    return 0;
  }

  uint32_t abs_speed = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)speed;
  if (abs_speed == 0) {
    return 0;
  }

  uint32_t period = 1000000UL / abs_speed;
  if (period > MINIMUN_TIMER_PERIOD) {
    period = MINIMUN_TIMER_PERIOD;
  }
  if (period < 2) {
    period = 2;
  }
  return period;
}

static void setDriverEnabled(bool enabled) {
  digitalWrite(CNC_ENABLE_PIN, enabled ? LOW : HIGH);
}

static void pulseStepPin(uint8_t pin, volatile int32_t &position, int8_t direction) {
  if (direction == 0 || emergency_stop_active) {
    return;
  }

  gpio_set_level((gpio_num_t)pin, 1);
  delayMicroseconds(2);
  gpio_set_level((gpio_num_t)pin, 0);
  position += direction;
}

static void motor1TimerCallback(void *arg) {
  (void)arg;
  pulseStepPin(X_STEP_PIN, position_M1, dir_M1);
}

static void motor2TimerCallback(void *arg) {
  (void)arg;
  pulseStepPin(Y_STEP_PIN, position_M2, dir_M2);
}

static void restartAxisTimer(esp_timer_handle_t timer, bool &running, uint32_t period_us) {
  if (timer == nullptr || period_us == 0 || emergency_stop_active) {
    if (timer != nullptr && running) {
      esp_timer_stop(timer);
      running = false;
    }
    return;
  }

  if (running) {
    esp_timer_stop(timer);
    running = false;
  }

  esp_timer_start_periodic(timer, period_us);
  running = true;
}

bool initializeMotionHardware()
{
  esp_timer_create_args_t motor1_args = {};
  motor1_args.callback = &motor1TimerCallback;
  motor1_args.name = "motor1";

  esp_timer_create_args_t motor2_args = {};
  motor2_args.callback = &motor2TimerCallback;
  motor2_args.name = "motor2";

  if (esp_timer_create(&motor1_args, &motor1_timer) != ESP_OK)
  {
    return false;
  }
  if (esp_timer_create(&motor2_args, &motor2_timer) != ESP_OK)
  {
    return false;
  }
  return true;
}

void stopMotionTimers() {
  if (motor1_timer != nullptr && motor1_timer_running) {
    esp_timer_stop(motor1_timer);
    motor1_timer_running = false;
  }
  if (motor2_timer != nullptr && motor2_timer_running) {
    esp_timer_stop(motor2_timer);
    motor2_timer_running = false;
  }
}

void emergencyStop() {
  emergency_stop_active = true;
  stopMotionTimers();
  speed_M1 = 0;
  speed_M2 = 0;
  target_speed_M1 = 0;
  target_speed_M2 = 0;
  dir_M1 = 0;
  dir_M2 = 0;
  working = false;
  setDriverEnabled(false);
}

void stopActiveMovement()
{
  stopMotionTimers();
  speed_M1 = 0;
  speed_M2 = 0;
  dir_M1 = 0;
  dir_M2 = 0;
  target_position_M1 = position_M1;
  target_position_M2 = position_M2;
  target_angleA1 = position_M1 / M1_AXIS_STEPS_PER_UNIT;
  target_angleA2 = position_M2 / M2_AXIS_STEPS_PER_UNIT;
  working = false;
}

void zeroCurrentPosition() {
  stopMotionTimers();
  position_M1 = 0;
  position_M2 = 0;
  target_position_M1 = 0;
  target_position_M2 = 0;
  speed_M1 = 0;
  speed_M2 = 0;
  dir_M1 = 0;
  dir_M2 = 0;
  working = false;
}

void setMotionLimits(int speed_percent, int acceleration_percent)
{
  configSpeed((int)((MAX_SPEED_M1 * float(speed_percent)) / 100.0f), (int)((MAX_SPEED_M2 * float(speed_percent)) / 100.0f));
  configAcceleration((int)((MAX_ACCEL_M1 * float(acceleration_percent)) / 100.0f), (int)((MAX_ACCEL_M2 * float(acceleration_percent)) / 100.0f));
  setSpeedAcc();
}

void configSpeed(int target_sM1, int target_sM2) {
  target_sM1 = constrain(target_sM1, 0, MAX_SPEED_M1);
  target_sM2 = constrain(target_sM2, 0, MAX_SPEED_M2);

  config_speed_M1 = target_sM1;
  config_speed_M2 = target_sM2;
}

void configAcceleration(int target_acc1, int target_acc2) {
  target_acc1 = constrain(target_acc1, MIN_ACCEL_M1, MAX_ACCEL_M1);
  target_acc2 = constrain(target_acc2, MIN_ACCEL_M2, MAX_ACCEL_M2);

  config_acceleration_M1 = target_acc1;
  config_acceleration_M2 = target_acc2;
}

void setSpeedAcc() {
  target_speed_M1 = config_speed_M1;
  target_speed_M2 = config_speed_M2;
  target_acceleration_M1 = config_acceleration_M1;
  target_acceleration_M2 = config_acceleration_M2;
}

void setAxis1(float angleA1) {
  angleA1 = constrain(angleA1, ROBOT_MIN_A1, ROBOT_MAX_A1);
  target_angleA1 = angleA1;
  target_position_M1 = (int32_t)(angleA1 * M1_AXIS_STEPS_PER_UNIT);
  Serial.print("A1:");
  Serial.print(angleA1);
  Serial.print(",");
  Serial.println(target_position_M1);
}

void setAxis2(float angleA2) {
  angleA2 = constrain(angleA2, ROBOT_MIN_A2, ROBOT_MAX_A2);
  target_angleA2 = angleA2;
  target_position_M2 = (int32_t)(angleA2 * M2_AXIS_STEPS_PER_UNIT);
  Serial.print("A2:");
  Serial.print(angleA2);
  Serial.print(",");
  Serial.println(target_position_M2);
}

void setAxisTarget(float axis1_degrees, float axis2_degrees)
{
  setAxis1(axis1_degrees);
  setAxis2(axis2_degrees);
  setSpeedAcc();
  working = true;
}

void consumeCommand(const RobotCommand &command)
{
  if (command.type == COMMAND_NONE)
  {
    return;
  }

  if (emergency_stop_active && command.type != COMMAND_EMERGENCY_STOP)
  {
    return;
  }

  switch (command.type)
  {
  case COMMAND_SET_AXIS_TARGET:
    setAxisTarget(command.axis1_degrees, command.axis2_degrees);
    break;
  case COMMAND_SET_MOTION_LIMITS:
    setMotionLimits(command.speed_percent, command.acceleration_percent);
    break;
  case COMMAND_ZERO_CURRENT_POSITION:
    zeroCurrentPosition();
    setSpeedAcc();
    break;
  case COMMAND_EMERGENCY_STOP:
    emergencyStop();
    digitalWrite(STATUS_LED_PIN, LOW);
    break;
  case COMMAND_STOP_ACTIVE_MOVEMENT:
    stopActiveMovement();
    break;
  case COMMAND_NONE:
    break;
  }
}

void setMotorM1Speed(int32_t tspeed, int16_t dt, int16_t overshoot_comp) {
  tspeed = constrain(tspeed, -MAX_SPEED_M1, MAX_SPEED_M1);
  overshoot_comp = constrain(overshoot_comp, -10, 10);

  int32_t accel = ((int64_t)acceleration_M1 * dt) / 1000 + overshoot_comp;
  if (accel < 1) {
    accel = 1;
  }

  if ((tspeed - speed_M1) > accel) {
    speed_M1 += accel;
  } else if ((speed_M1 - tspeed) > accel) {
    speed_M1 -= accel;
  } else {
    speed_M1 = tspeed;
  }

  if (speed_M1 == 0 || emergency_stop_active) {
    dir_M1 = 0;
    restartAxisTimer(motor1_timer, motor1_timer_running, 0);
    return;
  }

#ifdef INVERT_M1_AXIS
  if (speed_M1 > 0) {
    digitalWrite(X_DIR_PIN, LOW);
    dir_M1 = 1;
  } else {
    digitalWrite(X_DIR_PIN, HIGH);
    dir_M1 = -1;
  }
#else
  if (speed_M1 > 0) {
    digitalWrite(X_DIR_PIN, HIGH);
    dir_M1 = 1;
  } else {
    digitalWrite(X_DIR_PIN, LOW);
    dir_M1 = -1;
  }
#endif

  setDriverEnabled(true);
  restartAxisTimer(motor1_timer, motor1_timer_running, speedToPeriodUs(speed_M1));
}

void setMotorM2Speed(int32_t tspeed, int16_t dt, int16_t overshoot_comp) {
  tspeed = constrain(tspeed, -MAX_SPEED_M2, MAX_SPEED_M2);
  overshoot_comp = constrain(overshoot_comp, -10, 10);

  int32_t accel = ((int64_t)acceleration_M2 * dt) / 1000 + overshoot_comp;
  if (accel < 1) {
    accel = 1;
  }

  if ((tspeed - speed_M2) > accel) {
    speed_M2 += accel;
  } else if ((speed_M2 - tspeed) > accel) {
    speed_M2 -= accel;
  } else {
    speed_M2 = tspeed;
  }

  if (speed_M2 == 0 || emergency_stop_active) {
    dir_M2 = 0;
    restartAxisTimer(motor2_timer, motor2_timer_running, 0);
    return;
  }

#ifdef INVERT_M2_AXIS
  if (speed_M2 > 0) {
    digitalWrite(Y_DIR_PIN, LOW);
    dir_M2 = 1;
  } else {
    digitalWrite(Y_DIR_PIN, HIGH);
    dir_M2 = -1;
  }
#else
  if (speed_M2 > 0) {
    digitalWrite(Y_DIR_PIN, HIGH);
    dir_M2 = 1;
  } else {
    digitalWrite(Y_DIR_PIN, LOW);
    dir_M2 = -1;
  }
#endif

  setDriverEnabled(true);
  restartAxisTimer(motor2_timer, motor2_timer_running, speedToPeriodUs(speed_M2));
}

void positionControl(int dt) {
  int64_t temp;

  if (emergency_stop_active) {
    stopMotionTimers();
    working = false;
    return;
  }

  acceleration_M1 = target_acceleration_M1;
  acceleration_M2 = target_acceleration_M2;

  temp = ((int64_t)speed_M1 * speed_M1);
  temp = temp / (2000L * max(1L, (long)acceleration_M1));
  pos_stop_M1 = position_M1 + sign(speed_M1) * temp;

  if (target_position_M1 > position_M1) {
    if (pos_stop_M1 >= target_position_M1) {
      int32_t overshoot = pos_stop_M1 - target_position_M1;
      if (overshoot > overshoot_compensation) {
        setMotorM1Speed(0, dt, overshoot / overshoot_compensation);
      }
      M1stopping = true;
      setMotorM1Speed(0, dt, 0);
    } else {
      M1stopping = false;
      setMotorM1Speed(target_speed_M1, dt, 0);
    }
  } else if (target_position_M1 < position_M1) {
    if (pos_stop_M1 <= target_position_M1) {
      int32_t overshoot = target_position_M1 - pos_stop_M1;
      if (overshoot > overshoot_compensation) {
        setMotorM1Speed(0, dt, overshoot / overshoot_compensation);
      }
      M1stopping = true;
      setMotorM1Speed(0, dt, 0);
    } else {
      M1stopping = false;
      setMotorM1Speed(-target_speed_M1, dt, 0);
    }
  } else {
    M1stopping = false;
    setMotorM1Speed(0, dt, 0);
  }

  temp = ((int64_t)speed_M2 * speed_M2);
  temp = temp / (2000L * max(1L, (long)acceleration_M2));
  pos_stop_M2 = position_M2 + sign(speed_M2) * temp;

  if (target_position_M2 > position_M2) {
    if (pos_stop_M2 >= target_position_M2) {
      int32_t overshoot = pos_stop_M2 - target_position_M2;
      if (overshoot > overshoot_compensation) {
        setMotorM2Speed(0, dt, overshoot / overshoot_compensation);
      }
      M2stopping = true;
      setMotorM2Speed(0, dt, 0);
    } else {
      M2stopping = false;
      setMotorM2Speed(target_speed_M2, dt, 0);
    }
  } else if (target_position_M2 < position_M2) {
    if (pos_stop_M2 <= target_position_M2) {
      int32_t overshoot = target_position_M2 - pos_stop_M2;
      if (overshoot > overshoot_compensation) {
        setMotorM2Speed(0, dt, overshoot / overshoot_compensation);
      }
      M2stopping = true;
      setMotorM2Speed(0, dt, 0);
    } else {
      M2stopping = false;
      setMotorM2Speed(-target_speed_M2, dt, 0);
    }
  } else {
    M2stopping = false;
    setMotorM2Speed(0, dt, 0);
  }

  working = (dir_M1 != 0) || (dir_M2 != 0);
}
