#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <WiFi.h>

#include "HomingStateMachine.h"
#include "BoardConfiguration.h"
#include "CommandSystem.h"
#include "MotionSettings.h"
#include "MotorAdapter.h"
#include "secrets.h"
#include "StallDetector.h"
#include "TestController.h"
#include "WebCommandInput.h"

// TMC2209 sensorless homing for UM FeatherS2.
// This sketch confirms stalls by comparing SG_RESULT, read over UART, to the
// configured StallGuard threshold.

constexpr uint32_t CONSOLE_BAUD = 115200;
constexpr uint32_t TMC_UART_BAUD = 115200;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr uint8_t X_TMC_ADDRESS = 0;
constexpr uint8_t Y_TMC_ADDRESS = 1;

// Set these four values from the carrier-board schematic, motor datasheet, and
// a no-load StallGuard calibration. Zero keeps the sketch in a safe idle state.
constexpr float R_SENSE_OHMS = 0.11f;
constexpr uint16_t MOTOR_RMS_CURRENT_MA = 400;
constexpr uint32_t STALLGUARD_TCOOLTHRS = 0xFFFFF;
constexpr uint8_t STALLGUARD_SGTHRS = 45;

// If range (given in full steps) is known, set these values to avoid a range test. Zero keeps the sketch in a safe idle state.
constexpr uint32_t X_AXIS_RANGE_FULLSTEP = 512;
constexpr uint32_t Y_AXIS_RANGE_FULLSTEP = 704;

constexpr uint16_t DEFAULT_RUNTIME_MICROSTEPS = 8;
constexpr uint32_t HOMING_SPEED_HZ = 800;
constexpr uint32_t HOMING_ACCELERATION = 1600;
constexpr uint16_t HOMING_MICROSTEPS = 8;
constexpr int32_t ARMING_STEPS = 200;
constexpr int32_t MAX_SEEK_STEPS = 1000000;
constexpr int32_t BACKOFF_STEPS = 50;
constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 500;
constexpr uint32_t STALL_SAMPLE_INTERVAL_MS = 5;
constexpr uint8_t STALL_CONFIRM_SAMPLES = 1;
constexpr int32_t SEEK_SETTLE_STEPS = 800;
constexpr uint32_t RANGE_TEST_SPEED_HZ = 12000;
constexpr uint32_t RANGE_TEST_ACCELERATION = 12000;
constexpr uint8_t RANGE_TEST_CYCLES = 4;
constexpr uint32_t PATTERN_TEST_SPEED_HZ = 12000;
constexpr uint32_t PATTERN_TEST_ACCELERATION = 12000;
constexpr uint8_t PATTERN_TEST_DEFAULT_LOOPS = 2;
constexpr uint8_t PATTERN_MARGIN_PERCENT = 8;

RuntimeMotionSettings runtimeMotionSettings = {
    DEFAULT_RUNTIME_MICROSTEPS,
    PATTERN_TEST_SPEED_HZ,
    PATTERN_TEST_ACCELERATION,
};

HardwareSerial &tmcSerial = Serial1;
TMC2209Stepper xDriver(&tmcSerial, R_SENSE_OHMS, X_TMC_ADDRESS);
TMC2209Stepper yDriver(&tmcSerial, R_SENSE_OHMS, Y_TMC_ADDRESS);
FastAccelStepperEngine engine;
FastAccelStepper *xStepper = NULL;
FastAccelStepper *yStepper = NULL;

Axis xAxis = {"X", 0, 0};
Axis yAxis = {"Y", 0, 0};
MotorAdapter xMotor(xAxis, xDriver, xStepper, BOARD_CONFIGURATION.xStepPin, BOARD_CONFIGURATION.xDirPin);
MotorAdapter yMotor(yAxis, yDriver, yStepper, BOARD_CONFIGURATION.yStepPin, BOARD_CONFIGURATION.yDirPin);
MotorAdapter *activeMotor = &xMotor;

StallDetector stallDetector(STALL_SAMPLE_INTERVAL_MS, STALL_CONFIRM_SAMPLES, SEEK_SETTLE_STEPS);

bool configurationIsComplete();
void printDriverSample();
bool connectWifi();

const HomingStateMachineConfig homingConfig = {
    ARMING_STEPS,
    MAX_SEEK_STEPS,
    BACKOFF_STEPS,
    DIAGNOSTIC_INTERVAL_MS,
    static_cast<int32_t>(X_AXIS_RANGE_FULLSTEP *HOMING_MICROSTEPS),
    static_cast<int32_t>(Y_AXIS_RANGE_FULLSTEP *HOMING_MICROSTEPS),
    HOMING_SPEED_HZ,
    HOMING_ACCELERATION,
    HOMING_MICROSTEPS,
};

const TestControllerConfig testConfig = {
    HOMING_SPEED_HZ,
    HOMING_ACCELERATION,
    RANGE_TEST_SPEED_HZ,
    RANGE_TEST_ACCELERATION,
    RANGE_TEST_CYCLES,
    PATTERN_TEST_SPEED_HZ,
    PATTERN_TEST_ACCELERATION,
    PATTERN_TEST_DEFAULT_LOOPS,
    PATTERN_MARGIN_PERCENT,
};

TestController testController(testConfig, runtimeMotionSettings, activeMotor, xMotor, yMotor);

HomingStateMachine homing(
    homingConfig,
    activeMotor,
    xMotor,
    yMotor,
    stallDetector,
    testController,
    runtimeMotionSettings,
    printDriverSample);

CommandCatalog commandCatalog;
SerialCommandInput serialCommandInput(Serial, commandCatalog);
CommandDispatcher commandDispatcher(
    homing,
    testController,
    xMotor,
    yMotor,
    runtimeMotionSettings,
    commandCatalog,
    Serial,
    configurationIsComplete,
    printDriverSample);

WebCommandInput webCommandInput(
    commandDispatcher,
    commandCatalog,
    homing,
    testController,
    xMotor,
    yMotor,
    runtimeMotionSettings);

bool webApiEnabled = false;

bool configurationIsComplete()
{
    return R_SENSE_OHMS > 0.0f && MOTOR_RMS_CURRENT_MA > 0 &&
           STALLGUARD_TCOOLTHRS > 0 && STALLGUARD_SGTHRS > 0 &&
           MAX_SEEK_STEPS > ARMING_STEPS && BACKOFF_STEPS > 0;
}

void printDriverSample()
{
    Serial.print(F("[SG] axis="));
    Serial.print(activeMotor->axisName());
    Serial.print(F(" sg_result="));
    Serial.print(activeMotor->sgResult());
    Serial.print(F(" sgthrs="));
    Serial.println(activeMotor->sgThreshold());
}

void disableAxis()
{
    activeMotor->disableOutputs();
}

bool setupDrivers()
{
    tmcSerial.begin(TMC_UART_BAUD, SERIAL_8N1, BOARD_CONFIGURATION.tmcRxPin, BOARD_CONFIGURATION.tmcTxPin);
    const bool configurationComplete = configurationIsComplete();
    return xMotor.configureDriver(MOTOR_RMS_CURRENT_MA, runtimeMotionSettings.microsteps, STALLGUARD_TCOOLTHRS, STALLGUARD_SGTHRS, configurationComplete) &&
           yMotor.configureDriver(MOTOR_RMS_CURRENT_MA, runtimeMotionSettings.microsteps, STALLGUARD_TCOOLTHRS, STALLGUARD_SGTHRS, configurationComplete);
}

bool setupMotion()
{
    engine.init(BOARD_CONFIGURATION.fastAccelStepperCore);
    return xMotor.setupMotion(engine, BOARD_CONFIGURATION.enablePin, runtimeMotionSettings.speedHz, runtimeMotionSettings.acceleration) &&
           yMotor.setupMotion(engine, BOARD_CONFIGURATION.enablePin, runtimeMotionSettings.speedHz, runtimeMotionSettings.acceleration);
}

bool connectWifi()
{
    if (WIFI_SSID == nullptr || WIFI_SSID[0] == '\0')
    {
        Serial.println(F("WiFi disabled: set WIFI_SSID in secrets.h."));
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print(F("WiFi: connecting to "));
    Serial.println(WIFI_SSID);

    const uint32_t startedAtMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startedAtMs < WIFI_CONNECT_TIMEOUT_MS)
    {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(F("WiFi: connection failed; Web API disabled."));
        return false;
    }

    Serial.print(F("WiFi connected. Web API: http://"));
    Serial.println(WiFi.localIP());
    return true;
}

void setup()
{
    Serial.begin(CONSOLE_BAUD);
    delay(500);

    if (!setupDrivers() || !setupMotion())
    {
        homing.setFaulted();
        return;
    }

    disableAxis();
    Serial.print(F("Board: "));
    Serial.println(BOARD_CONFIGURATION.name);
    Serial.println(F("Ready. Send 'p' for the command catalog."));
    commandCatalog.printTo(Serial);

    if (connectWifi())
    {
        webCommandInput.begin();
        webApiEnabled = true;
        Serial.println(F("Web API ready."));
    }
}

void loop()
{
    while (Serial.available() > 0)
    {
        Command command = {};
        if (serialCommandInput.read(command))
        {
            commandDispatcher.dispatch(command);
        }
    }

    homing.update();
    testController.update();

    if (webApiEnabled)
    {
        webCommandInput.handleClient();
    }
}
