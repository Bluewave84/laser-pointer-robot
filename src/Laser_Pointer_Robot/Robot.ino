// ESP32 motion control for the laser pointer platform.

#include <Arduino.h>
#include <driver/gpio.h>
#include "Command.h"

void emergencyStop() {
  emergency_stop_active = true;
  
  if (stepper_M1) {
    stepper_M1->forceStop();
  }
  if (stepper_M2) {
    stepper_M2->forceStop();
  }
  
  config_speed_M1 = 0;
  config_speed_M2 = 0;
  config_acceleration_M1 = MAX_ACCEL_M1;
  config_acceleration_M2 = MAX_ACCEL_M2;
  
  digitalWrite(CNC_ENABLE_PIN, HIGH);
  working = false;
}

void stopActiveMovement()
{
  if (stepper_M1) {
    stepper_M1->stopMove();
  }
  if (stepper_M2) {
    stepper_M2->stopMove();
  }
  
  working = false;
}

void zeroCurrentPosition() {
  if (stepper_M1) {
    stepper_M1->setCurrentPosition(0);
  }
  if (stepper_M2) {
    stepper_M2->setCurrentPosition(0);
  }
  
  target_angleA1 = 0;
  target_angleA2 = 0;
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
  if (stepper_M1) {
    stepper_M1->setMaxSpeed(config_speed_M1);
    stepper_M1->setAcceleration(config_acceleration_M1);
  }
  if (stepper_M2) {
    stepper_M2->setMaxSpeed(config_speed_M2);
    stepper_M2->setAcceleration(config_acceleration_M2);
  }
}

void setAxis1(float angleA1) {
  angleA1 = constrain(angleA1, ROBOT_MIN_A1, ROBOT_MAX_A1);
  target_angleA1 = angleA1;
  int32_t target_position = (int32_t)(angleA1 * M1_AXIS_STEPS_PER_UNIT);
  
  if (stepper_M1) {
    stepper_M1->moveTo(target_position);
  }
  
  Serial.print("A1:");
  Serial.print(angleA1);
  Serial.print(",");
  Serial.println(target_position);
}

void setAxis2(float angleA2) {
  angleA2 = constrain(angleA2, ROBOT_MIN_A2, ROBOT_MAX_A2);
  target_angleA2 = angleA2;
  int32_t target_position = (int32_t)(angleA2 * M2_AXIS_STEPS_PER_UNIT);
  
  if (stepper_M2) {
    stepper_M2->moveTo(target_position);
  }
  
  Serial.print("A2:");
  Serial.print(angleA2);
  Serial.print(",");
  Serial.println(target_position);
}

void setAxisTarget(float axis1_degrees, float axis2_degrees)
{
  setAxis1(axis1_degrees);
  setAxis2(axis2_degrees);
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
