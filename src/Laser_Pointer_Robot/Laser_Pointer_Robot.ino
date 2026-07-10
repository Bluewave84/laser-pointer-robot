// ESP32 CNC laser pointer firmware.

#include <Arduino.h>
#include "Configuration.h"
#include "Command.h"

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

  beginCommandInput();
  
  // Initialize FastAccelStepper engine
  engine.init();
  
  // Configure stepper M1 (X axis)
  stepper_M1 = engine.stepperConnectToPin(X_STEP_PIN);
  if (stepper_M1) {
    stepper_M1->setDirectionPin(X_DIR_PIN);
    stepper_M1->setSpeedInHz(config_speed_M1);
    stepper_M1->setAcceleration(config_acceleration_M1);
    stepper_M1->setCurrentPosition(0);
  }
  
  // Configure stepper M2 (Y axis)
  stepper_M2 = engine.stepperConnectToPin(Y_STEP_PIN);
  if (stepper_M2) {
    stepper_M2->setDirectionPin(Y_DIR_PIN);
    stepper_M2->setSpeedInHz(config_speed_M2);
    stepper_M2->setAcceleration(config_acceleration_M2);
    stepper_M2->setCurrentPosition(0);
  }

  configSpeed(MAX_SPEED_M1, MAX_SPEED_M2);
  configAcceleration(MAX_ACCEL_M1, MAX_ACCEL_M2);
  setSpeedAcc();

  emergency_stop_active = false;
  digitalWrite(CNC_ENABLE_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, HIGH);

  slow_timer_old = millis();
  laser_timer_old = millis();

  Serial.println("Ready.");
}

void loop() {
  pollCommandInput();

  RobotCommand command;
  if (takeCommand(command))
  {
    consumeCommand(command);
  }

  // Check if any motor is still moving
  if (stepper_M1 && stepper_M2) {
    bool m1_moving = stepper_M1->isRunning();
    bool m2_moving = stepper_M2->isRunning();
    working = m1_moving || m2_moving;
    
    // Enable driver if any motor is moving, disable if both idle
    digitalWrite(CNC_ENABLE_PIN, working ? LOW : HIGH);
  }
}
