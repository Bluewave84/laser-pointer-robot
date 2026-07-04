// ESP32 network input and message parsing.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"

static WiFiUDP commandUdp;
static const uint16_t COMMAND_PORT = 2222;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;

static bool hasPendingCommand()
{
  return pending_command.type != COMMAND_NONE;
}

static void emitCommand(const RobotCommand &command)
{
  pending_command = command;
}

static void feedMessageByte(uint8_t interface_id, uint8_t value)
{
  for (uint8_t i = 0; i < (MSGMAXLEN - 1); i++)
  {
    MsgBuffer[i] = MsgBuffer[i + 1];
  }
  MsgBuffer[MSGMAXLEN - 1] = value;
  ParseMsg(interface_id);
}

bool NetworkBegin()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi SSID: ");
  Serial.println(WIFI_SSID);

  uint32_t start_time = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start_time) < WIFI_CONNECT_TIMEOUT_MS)
  {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wi-Fi connection failed");
    return false;
  }

  commandUdp.begin(COMMAND_PORT);

  Serial.print("Wi-Fi connected. IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("WiFi UDP port: ");
  Serial.println(COMMAND_PORT);
  return true;
}

int32_t ExtractParamInt4b(uint8_t pos)
{
  union
  {
    unsigned char Buff[4];
    int32_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 3];
  u.Buff[1] = (unsigned char)MsgBuffer[pos + 2];
  u.Buff[2] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[3] = (unsigned char)MsgBuffer[pos];
  return u.d;
}

int16_t ExtractParamInt2b(uint8_t pos)
{
  union
  {
    unsigned char Buff[2];
    int16_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[1] = (unsigned char)MsgBuffer[pos];
  return u.d;
}

int32_t ExtractParamString6b(uint8_t pos)
{
  char Buff[7];

  for (uint8_t i = 0; i < 6; i++)
  {
    Buff[i] = (char)MsgBuffer[pos + i];
  }
  Buff[6] = 0;
  return atoi(Buff);
}

void MsgRead()
{
  int packetSize = commandUdp.parsePacket();
  while (packetSize > 0)
  {
    while (commandUdp.available() > 0)
    {
      feedMessageByte(1, (uint8_t)commandUdp.read());
    }
    packetSize = commandUdp.parsePacket();
  }
}

void USBMsgRead()
{
  while (Serial.available() > 0)
  {
    feedMessageByte(0, (uint8_t)Serial.read());
  }
}

void ParseMsg(uint8_t interface)
{
  (void)interface;

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'H'))
  {
    Serial.println("->MSG: JJAH: STOP ACTIVE MOVEMENT");
    emitCommand({COMMAND_STOP_ACTIVE_MOVEMENT, 0.0f, 0.0f, 0, 0});
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'T'))
  {
    Serial.print("->MSG: JJAT:");
    int32_t axis1_value = ExtractParamString6b(4);
    int32_t axis2_value = ExtractParamString6b(11);
    Serial.print(axis1_value);
    Serial.print(" ");
    Serial.println(axis2_value);
    emitCommand({COMMAND_SET_AXIS_TARGET, axis1_value / 100.0f, axis2_value / 100.0f, 0, 0});
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'M'))
  {
    Serial.print("->MSG: JJAM:");
    int16_t axis1_value = ExtractParamInt2b(4);
    int16_t axis2_value = ExtractParamInt2b(6);
    int16_t first_unused_value = ExtractParamInt2b(8);
    Serial.print(axis1_value);
    Serial.print(" ");
    Serial.print(axis2_value);
    Serial.print(" ");
    Serial.println(first_unused_value);
    emitCommand({COMMAND_SET_AXIS_TARGET, axis1_value / 100.0f, axis2_value / 100.0f, 0, 0});
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'S'))
  {
    Serial.print("->MSG: JJAS:");
    int16_t speed_percent = ExtractParamInt2b(4);
    int16_t acceleration_percent = ExtractParamInt2b(8);
    int16_t trajectory_speed = ExtractParamInt2b(12);
    Serial.print(" SPEED XY:");
    Serial.print(speed_percent);
    Serial.print(" ACC XY:");
    Serial.print(acceleration_percent);
    Serial.print(" TRAJ S:");
    Serial.println(trajectory_speed);
    emitCommand({COMMAND_SET_MOTION_LIMITS, 0.0f, 0.0f, speed_percent, acceleration_percent});
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'C'))
  {
    Serial.println("->MSG: JJAC:");
    emitCommand({COMMAND_ZERO_CURRENT_POSITION, 0.0f, 0.0f, 0, 0});
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'E'))
  {
    Serial.println("->MSG: JJAE:");
    emitCommand({COMMAND_EMERGENCY_STOP, 0.0f, 0.0f, 0, 0});
    return;
  }
}