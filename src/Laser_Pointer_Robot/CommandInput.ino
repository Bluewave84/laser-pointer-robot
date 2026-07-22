// ESP32 command input and message frame parsing.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <errno.h>
#include "Command.h"
#include "secrets.h"

static WiFiUDP commandUdp;
static WebServer commandServer(80);
static const uint16_t COMMAND_PORT = 2222;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static const uint8_t MESSAGE_FRAME_BUFFER_LENGTH = 20;
static const uint8_t CONTROL_MESSAGE_FRAME_LENGTH = 4;
static const uint8_t TEXT_AXIS_TARGET_MESSAGE_FRAME_LENGTH = 17;
static const uint8_t BINARY_AXIS_TARGET_MESSAGE_FRAME_LENGTH = 20;
static const uint8_t MOTION_LIMITS_MESSAGE_FRAME_LENGTH = 14;

static bool udp_transport_enabled = false;
static bool http_transport_enabled = false;
static RobotCommand pending_command = {COMMAND_NONE, 0.0f, 0.0f, 0, 0};
static uint8_t message_frame_buffer[MESSAGE_FRAME_BUFFER_LENGTH] = {0};

static void emitCommand(const RobotCommand &command)
{
    if (pending_command.type == COMMAND_EMERGENCY_STOP && command.type != COMMAND_EMERGENCY_STOP)
    {
        return;
    }
    pending_command = command;
}

static int32_t extractParamInt4b(uint8_t pos)
{
    union
    {
        unsigned char Buff[4];
        int32_t d;
    } u;
    u.Buff[0] = (unsigned char)message_frame_buffer[pos + 3];
    u.Buff[1] = (unsigned char)message_frame_buffer[pos + 2];
    u.Buff[2] = (unsigned char)message_frame_buffer[pos + 1];
    u.Buff[3] = (unsigned char)message_frame_buffer[pos];
    return u.d;
}

static int16_t extractParamInt2b(uint8_t pos)
{
    union
    {
        unsigned char Buff[2];
        int16_t d;
    } u;
    u.Buff[0] = (unsigned char)message_frame_buffer[pos + 1];
    u.Buff[1] = (unsigned char)message_frame_buffer[pos];
    return u.d;
}

static int32_t extractParamString6b(uint8_t pos)
{
    char Buff[7];

    for (uint8_t i = 0; i < 6; i++)
    {
        Buff[i] = (char)message_frame_buffer[pos + i];
    }
    Buff[6] = 0;
    return atoi(Buff);
}

static bool messageFrameMatches(uint8_t frame_start, char command_code)
{
    return (char(message_frame_buffer[frame_start]) == 'J') &&
           (char(message_frame_buffer[frame_start + 1]) == 'J') &&
           (char(message_frame_buffer[frame_start + 2]) == 'A') &&
           (char(message_frame_buffer[frame_start + 3]) == command_code);
}

static void parseMessageFrame()
{
    const uint8_t control_frame_start = MESSAGE_FRAME_BUFFER_LENGTH - CONTROL_MESSAGE_FRAME_LENGTH;
    const uint8_t text_axis_target_frame_start = MESSAGE_FRAME_BUFFER_LENGTH - TEXT_AXIS_TARGET_MESSAGE_FRAME_LENGTH;
    const uint8_t binary_axis_target_frame_start = MESSAGE_FRAME_BUFFER_LENGTH - BINARY_AXIS_TARGET_MESSAGE_FRAME_LENGTH;
    const uint8_t motion_limits_frame_start = MESSAGE_FRAME_BUFFER_LENGTH - MOTION_LIMITS_MESSAGE_FRAME_LENGTH;

    if (messageFrameMatches(control_frame_start, 'H'))
    {
        Serial.println("->MSG: JJAH: STOP ACTIVE MOVEMENT");
        emitCommand({COMMAND_STOP_ACTIVE_MOVEMENT, 0.0f, 0.0f, 0, 0});
        return;
    }

    if (messageFrameMatches(text_axis_target_frame_start, 'T'))
    {
        Serial.print("->MSG: JJAT:");
        int32_t axis1_value = extractParamString6b(text_axis_target_frame_start + 4);
        int32_t axis2_value = extractParamString6b(text_axis_target_frame_start + 11);
        Serial.print(axis1_value);
        Serial.print(" ");
        Serial.println(axis2_value);
        emitCommand({COMMAND_SET_AXIS_TARGET, axis1_value / 100.0f, axis2_value / 100.0f, 0, 0});
        return;
    }

    if (messageFrameMatches(binary_axis_target_frame_start, 'M'))
    {
        Serial.print("->MSG: JJAM:");
        int16_t axis1_value = extractParamInt2b(binary_axis_target_frame_start + 4);
        int16_t axis2_value = extractParamInt2b(binary_axis_target_frame_start + 6);
        int16_t first_unused_value = extractParamInt2b(binary_axis_target_frame_start + 8);
        Serial.print(axis1_value);
        Serial.print(" ");
        Serial.print(axis2_value);
        Serial.print(" ");
        Serial.println(first_unused_value);
        emitCommand({COMMAND_SET_AXIS_TARGET, axis1_value / 100.0f, axis2_value / 100.0f, 0, 0});
        return;
    }

    if (messageFrameMatches(motion_limits_frame_start, 'S'))
    {
        Serial.print("->MSG: JJAS:");
        int16_t speed_percent = extractParamInt2b(motion_limits_frame_start + 4);
        int16_t acceleration_percent = extractParamInt2b(motion_limits_frame_start + 8);
        int16_t trajectory_speed = extractParamInt2b(motion_limits_frame_start + 12);
        Serial.print(" SPEED XY:");
        Serial.print(speed_percent);
        Serial.print(" ACC XY:");
        Serial.print(acceleration_percent);
        Serial.print(" TRAJ S:");
        Serial.println(trajectory_speed);
        emitCommand({COMMAND_SET_MOTION_LIMITS, 0.0f, 0.0f, speed_percent, acceleration_percent});
        return;
    }

    if (messageFrameMatches(control_frame_start, 'C'))
    {
        Serial.println("->MSG: JJAC:");
        emitCommand({COMMAND_ZERO_CURRENT_POSITION, 0.0f, 0.0f, 0, 0});
        return;
    }

    if (messageFrameMatches(control_frame_start, 'E'))
    {
        Serial.println("->MSG: JJAE:");
        emitCommand({COMMAND_EMERGENCY_STOP, 0.0f, 0.0f, 0, 0});
        return;
    }
}

static void feedMessageByte(uint8_t value)
{
    for (uint8_t i = 0; i < (MESSAGE_FRAME_BUFFER_LENGTH - 1); i++)
    {
        message_frame_buffer[i] = message_frame_buffer[i + 1];
    }
    message_frame_buffer[MESSAGE_FRAME_BUFFER_LENGTH - 1] = value;
    parseMessageFrame();
}

static void pollUdpTransport()
{
    if (!udp_transport_enabled)
    {
        return;
    }

    int packetSize = commandUdp.parsePacket();
    while (packetSize > 0)
    {
        while (commandUdp.available() > 0)
        {
            feedMessageByte((uint8_t)commandUdp.read());
        }
        packetSize = commandUdp.parsePacket();
    }
}

static void pollSerialTransport()
{
    while (Serial.available() > 0)
    {
        feedMessageByte((uint8_t)Serial.read());
    }
}

static bool beginWifiConnection()
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
        Serial.println("Wi-Fi connection failed; transports disabled");
        return false;
    }

    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
}

static bool beginUdpTransport()
{
    if (!commandUdp.begin(COMMAND_PORT))
    {
        Serial.println("WiFi UDP start failed; UDP Transport disabled");
        return false;
    }

    Serial.print("WiFi UDP port: ");
    Serial.println(COMMAND_PORT);
    return true;
}

// Parses a string as a float, rejecting empty strings, non-numeric input,
// partial parses, out-of-range values (ERANGE), infinity, and NaN.
// Returns true and sets out to the parsed value on success; out is unchanged on failure.
static bool parseCommandFloat(const String &s, float &out)
{
    if (s.length() == 0)
    {
        return false;
    }
    const char *str = s.c_str();
    char *endptr;
    errno = 0;
    float value = strtof(str, &endptr);
    if (endptr == str || *endptr != '\0' || errno == ERANGE || isinf(value) || isnan(value))
    {
        return false;
    }
    out = value;
    return true;
}

static void handleHttpTarget()
{
    if (!commandServer.hasArg("x") || !commandServer.hasArg("y"))
    {
        commandServer.send(400, "text/plain", "Missing x or y parameter");
        return;
    }

    float axis1_degrees, axis2_degrees;
    if (!parseCommandFloat(commandServer.arg("x"), axis1_degrees) || !parseCommandFloat(commandServer.arg("y"), axis2_degrees))
    {
        commandServer.send(400, "text/plain", "Invalid numeric value for x or y parameter");
        return;
    }

    Serial.print("->HTTP /target: x=");
    Serial.print(axis1_degrees);
    Serial.print(" y=");
    Serial.println(axis2_degrees);

    emitCommand({COMMAND_SET_AXIS_TARGET, axis1_degrees, axis2_degrees, 0, 0});
    commandServer.send(200, "text/plain", "OK");
}

static bool beginHttpTransport()
{
    commandServer.on("/target", HTTP_GET, handleHttpTarget);
    commandServer.begin();
    Serial.println("HTTP server started on port 80");
    return true;
}

bool beginCommandInput()
{
    bool wifi_connected = beginWifiConnection();
    if (wifi_connected)
    {
        udp_transport_enabled = beginUdpTransport();
        http_transport_enabled = beginHttpTransport();
    }
    Serial.print("Transports enabled — UDP: ");
    Serial.print(udp_transport_enabled ? "yes" : "no");
    Serial.print(", HTTP: ");
    Serial.println(http_transport_enabled ? "yes" : "no");
    return true;
}

void pollCommandInput()
{
    pollUdpTransport();
    pollSerialTransport();
    if (http_transport_enabled)
    {
        commandServer.handleClient();
    }
}

bool takeCommand(RobotCommand &command)
{
    if (pending_command.type == COMMAND_NONE)
    {
        return false;
    }

    command = pending_command;
    pending_command = {COMMAND_NONE, 0.0f, 0.0f, 0, 0};
    return true;
}