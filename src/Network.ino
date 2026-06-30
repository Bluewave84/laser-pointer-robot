// ESP32 network input and message parsing.

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP commandUdp;
static const uint16_t COMMAND_PORT = 2222;
static const char *WIFI_PASSWORD = "87654321";

static void feedMessageByte(uint8_t interface_id, uint8_t value) {
  for (uint8_t i = 0; i < (MSGMAXLEN - 1); i++) {
    MsgBuffer[i] = MsgBuffer[i + 1];
  }
  MsgBuffer[MSGMAXLEN - 1] = value;
  ParseMsg(interface_id);
}

void NetworkBegin() {
  WiFi.mode(WIFI_AP);

  String suffix = String((uint32_t)(ESP.getEfuseMac() & 0xFFFF), HEX);
  suffix.toUpperCase();
  String apName = String("JJROBOTS_ESP32_") + suffix;

  if (!WiFi.softAP(apName.c_str(), WIFI_PASSWORD)) {
    Serial.println("WiFi AP start failed");
    return;
  }

  commandUdp.begin(COMMAND_PORT);

  Serial.print("WiFi AP SSID: ");
  Serial.println(apName);
  Serial.print("WiFi AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("WiFi UDP port: ");
  Serial.println(COMMAND_PORT);
}

int32_t ExtractParamInt4b(uint8_t pos) {
  union {
    unsigned char Buff[4];
    int32_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 3];
  u.Buff[1] = (unsigned char)MsgBuffer[pos + 2];
  u.Buff[2] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[3] = (unsigned char)MsgBuffer[pos];
  return u.d;
}

int16_t ExtractParamInt2b(uint8_t pos) {
  union {
    unsigned char Buff[2];
    int16_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[1] = (unsigned char)MsgBuffer[pos];
  return u.d;
}

int32_t ExtractParamString6b(uint8_t pos) {
  char Buff[7];

  for (uint8_t i = 0; i < 6; i++) {
    Buff[i] = (char)MsgBuffer[pos + i];
  }
  Buff[6] = 0;
  return atoi(Buff);
}

void MsgRead() {
  int packetSize = commandUdp.parsePacket();
  while (packetSize > 0) {
    while (commandUdp.available() > 0) {
      feedMessageByte(1, (uint8_t)commandUdp.read());
    }
    packetSize = commandUdp.parsePacket();
  }
}

void USBMsgRead() {
  while (Serial.available() > 0) {
    feedMessageByte(0, (uint8_t)Serial.read());
  }
}

void ParseMsg(uint8_t interface) {
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'H')) {
    Serial.println("->MSG: JJAH: HELLO!");
    working = false;
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'T')) {
    Serial.print("->MSG: JJAT:");
    iCH1 = ExtractParamString6b(4);
    iCH2 = ExtractParamString6b(11);
    iCH3 = 0;
    iCH4 = 0;
    iCH5 = 0;
    iCH6 = 0;
    iCH7 = 0;
    iCH8 = 0;
    Serial.print(iCH1);
    Serial.print(" ");
    Serial.println(iCH2);
    mode = 1;
    newMessage = 1;
    working = true;
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'M')) {
    Serial.print("->MSG: JJAM:");
    iCH1 = ExtractParamInt2b(4);
    iCH2 = ExtractParamInt2b(6);
    iCH3 = ExtractParamInt2b(8);
    iCH4 = ExtractParamInt2b(10);
    iCH5 = ExtractParamInt2b(12);
    iCH6 = ExtractParamInt2b(14);
    iCH7 = ExtractParamInt2b(16);
    iCH8 = ExtractParamInt2b(18);
    Serial.print(iCH1);
    Serial.print(" ");
    Serial.print(iCH2);
    Serial.print(" ");
    Serial.println(iCH3);
    mode = 1;
    newMessage = 1;
    working = true;
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'S')) {
    Serial.print("->MSG: JJAS:");
    iCH1 = ExtractParamInt2b(4);
    iCH2 = ExtractParamInt2b(6);
    iCH3 = ExtractParamInt2b(8);
    iCH4 = ExtractParamInt2b(10);
    iCH5 = ExtractParamInt2b(12);
    Serial.print(" SPEED XY:");
    Serial.print(iCH1);
    Serial.print(" ACC XY:");
    Serial.print(iCH3);
    Serial.print(" TRAJ S:");
    Serial.println(iCH5);

    configSpeed((int)((MAX_SPEED_M1 * float(iCH1)) / 100.0f), (int)((MAX_SPEED_M2 * float(iCH1)) / 100.0f));
    configAcceleration((int)((MAX_ACCEL_M1 * float(iCH3)) / 100.0f), (int)((MAX_ACCEL_M2 * float(iCH3)) / 100.0f));
    setSpeedAcc();
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'C')) {
    Serial.println("->MSG: JJAC:");
    mode = 5;
    newMessage = 1;
    working = true;
    return;
  }

  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'E')) {
    Serial.println("->MSG: JJAE:");
    mode = 4;
    newMessage = 1;
    working = true;
    return;
  }
}
