// 2 Axis robot project
// ESP32 CNC Shield V3 port

#include <stdint.h>
#include <math.h>

#define ROBOT_ABSOLUTE_MAX_M1 121
#define ROBOT_ABSOLUTE_MAX_M2 142

#define MICROSTEPPING 16
#define MOTOR_STEPS 200

#define M1_REDUCTION 4
#define M2_REDUCTION 3.2

#define M1_AXIS_STEPS_PER_UNIT (((MOTOR_STEPS * MICROSTEPPING) / 360.0) * M1_REDUCTION)
#define M2_AXIS_STEPS_PER_UNIT (((MOTOR_STEPS * MICROSTEPPING) / 360.0) * M2_REDUCTION)

#define MAX_ACCEL_M1 30
#define MAX_ACCEL_M2 30

#define MIN_ACCEL_M1 50
#define MIN_ACCEL_M2 50

#define MAX_SPEED_M1 16000
#define MAX_SPEED_M2 16000

#define MIN_SPEED_M1 200
#define MIN_SPEED_M2 200

#define ROBOT_MIN_A1 -3600.0
#define ROBOT_MIN_A2 -360.0
#define ROBOT_MAX_A1 360.0
#define ROBOT_MAX_A2 360.0

#define ROBOT_INITIAL_POSITION_M1 0
#define ROBOT_INITIAL_POSITION_M2 0

#define MINIMUN_TIMER_PERIOD 32000
#define MSGMAXLEN 20
#define NODATA -20000
#define STOP_TOLERANCE 5

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define X_STEP_PIN 26
#define X_DIR_PIN 27
#define Y_STEP_PIN 25
#define Y_DIR_PIN 33
#define CNC_ENABLE_PIN 32
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define STATUS_LED_PIN LED_BUILTIN

#define TELEMETRY "192.168.4.2"

struct angles {
  float A1;
  float A2;
};

enum CommandType
{
  COMMAND_NONE = 0,
  COMMAND_SET_AXIS_TARGET,
  COMMAND_SET_MOTION_LIMITS,
  COMMAND_ZERO_CURRENT_POSITION,
  COMMAND_EMERGENCY_STOP,
  COMMAND_STOP_ACTIVE_MOVEMENT
};

struct RobotCommand
{
  CommandType type;
  float axis1_degrees;
  float axis2_degrees;
  int speed_percent;
  int acceleration_percent;
};

long loop_counter;
int16_t slow_loop_counter;
long timeout_counter;
long wait_counter;
int16_t timestamp = 0;
int dt;
long timer_old;
long timer_value;
long slow_timer_old;
long slow_timer_value;
long laser_timer_old;
long laser_timer_value;
int debug_counter;
bool enable_udp_output = false;

volatile int32_t position_M1;
volatile int32_t position_M2;
bool working = false;
bool M1stopping = false;
bool M2stopping = false;

int8_t dir_M1 = 0;
int8_t dir_M2 = 0;
float target_angleA1 = 0;
float target_angleA2 = 0;
int32_t target_position_M1 = 0;
int32_t target_position_M2 = 0;
int32_t diff_M1 = 0;
int32_t diff_M2 = 0;
int32_t speed_M1 = 0;
int32_t speed_M2 = 0;
int32_t config_speed_M1 = MAX_SPEED_M1;
int32_t config_speed_M2 = MAX_SPEED_M2;
int32_t target_speed_M1 = MAX_SPEED_M1;
int32_t target_speed_M2 = MAX_SPEED_M2;

int32_t acceleration_M1 = MAX_ACCEL_M1;
int32_t acceleration_M2 = MAX_ACCEL_M2;
int32_t config_acceleration_M1 = MAX_ACCEL_M1;
int32_t config_acceleration_M2 = MAX_ACCEL_M2;
int32_t target_acceleration_M1 = MAX_ACCEL_M1;
int32_t target_acceleration_M2 = MAX_ACCEL_M2;

int32_t pos_stop_M1 = 0;
int32_t pos_stop_M2 = 0;
int16_t overshoot_compensation = 20;

float actual_angleA1 = 0;
float actual_angleA2 = 0;

long servo_counter;
int16_t servo_pos1;
int16_t servo_pos2;
bool servo1_ready = false;
bool servo2_ready = false;

RobotCommand pending_command = {COMMAND_NONE, 0.0f, 0.0f, 0, 0};
uint8_t MsgBuffer[MSGMAXLEN] = {0};

String MAC;
bool emergency_stop_active = false;

int16_t myAbs(int16_t param) {
  return (param < 0) ? -param : param;
}

long myAbsLong(long param) {
  return (param < 0) ? -param : param;
}

int sign(int32_t val) {
  if (val < 0) {
    return -1;
  }
  if (val > 0) {
    return 1;
  }
  return 0;
}
