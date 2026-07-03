// ESP32 CNC laser pointer firmware.

#include <Arduino.h>
#include "Configuration.h"

#define VERSION "ESP32 Laser Pointer v1"

void setup() {
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(CNC_ENABLE_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  digitalWrite(X_STEP_PIN, LOW);
  digitalWrite(Y_STEP_PIN, LOW);
  digitalWrite(X_DIR_PIN, LOW);
  digitalWrite(Y_DIR_PIN, LOW);
  digitalWrite(CNC_ENABLE_PIN, HIGH);
  digitalWrite(STATUS_LED_PIN, LOW);

  delay(100);
  Serial.begin(115200);
  delay(500);

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
  Serial.print("ENABLE: ");
  Serial.println(CNC_ENABLE_PIN);

  NetworkBegin();
  initializeMotionHardware();

  position_M1 = ROBOT_INITIAL_POSITION_M1 * M1_AXIS_STEPS_PER_UNIT;
  position_M2 = ROBOT_INITIAL_POSITION_M2 * M2_AXIS_STEPS_PER_UNIT;
  target_position_M1 = position_M1;
  target_position_M2 = position_M2;

  configSpeed(MAX_SPEED_M1, MAX_SPEED_M2);
  configAcceleration(MAX_ACCEL_M1, MAX_ACCEL_M2);
  setSpeedAcc();

  emergency_stop_active = false;
  digitalWrite(CNC_ENABLE_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, HIGH);

  timer_old = micros();
  slow_timer_old = millis();
  laser_timer_old = millis();

  Serial.println("Ready.");
}

void loop() {
  MsgRead();
  USBMsgRead();

  if (hasPendingCommand())
  {
    consumeCommand(pending_command);
    pending_command.type = COMMAND_NONE;
  }

  timer_value = micros();
  dt = (int)(timer_value - timer_old);
  if (dt >= 1000) {
    timer_old = timer_value;
    positionControl(1000);
  }
}
