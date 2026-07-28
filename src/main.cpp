#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>

#include "HomingStateMachine.h"
#include "MotorAdapter.h"
#include "StallDetector.h"

// TMC2209 sensorless homing for UM FeatherS2.
// This sketch confirms stalls by comparing SG_RESULT, read over UART, to the
// configured StallGuard threshold.

constexpr uint32_t CONSOLE_BAUD = 115200;
constexpr uint32_t TMC_UART_BAUD = 115200;
constexpr uint8_t X_TMC_ADDRESS = 0;
constexpr uint8_t Y_TMC_ADDRESS = 1;

// Verified UM FeatherS2 wiring.
constexpr int TMC_RX_PIN = 44;
constexpr int TMC_TX_PIN = 43;
constexpr uint8_t X_STEP_PIN = 17;
constexpr uint8_t X_DIR_PIN = 18;
constexpr uint8_t Y_STEP_PIN = 5;
constexpr uint8_t Y_DIR_PIN = 6;
constexpr uint8_t ENN_PIN = 12;

// Set these four values from the carrier-board schematic, motor datasheet, and
// a no-load StallGuard calibration. Zero keeps the sketch in a safe idle state.
constexpr float R_SENSE_OHMS = 0.11f;
constexpr uint16_t MOTOR_RMS_CURRENT_MA = 400;
constexpr uint32_t STALLGUARD_TCOOLTHRS = 0xFFFFF;
constexpr uint8_t STALLGUARD_SGTHRS = 45;

// If range (given in full steps) is known, set these values to avoid a range test. Zero keeps the sketch in a safe idle state.
constexpr uint32_t X_AXIS_RANGE_FULLSTEP = 512;
constexpr uint32_t Y_AXIS_RANGE_FULLSTEP = 704;

constexpr uint16_t MICROSTEPS = 8;
constexpr uint32_t HOMING_SPEED_HZ = 800;
constexpr uint32_t HOMING_ACCELERATION = 1600;
constexpr int32_t ARMING_STEPS = 200;
constexpr int32_t MAX_SEEK_STEPS = 1000000;
constexpr int32_t BACKOFF_STEPS = 50;
constexpr uint32_t DIAGNOSTIC_INTERVAL_MS = 500;
constexpr uint32_t STALL_SAMPLE_INTERVAL_MS = 5;
constexpr uint8_t STALL_CONFIRM_SAMPLES = 1;
constexpr int32_t SEEK_SETTLE_STEPS = 800;
constexpr uint32_t RANGE_TEST_SPEED_HZ = 8000;
constexpr uint32_t RANGE_TEST_ACCELERATION = 16000;
constexpr uint8_t RANGE_TEST_CYCLES = 4;
constexpr uint32_t PATTERN_TEST_SPEED_HZ = 4800;
constexpr uint32_t PATTERN_TEST_ACCELERATION = 12000;
constexpr uint8_t PATTERN_TEST_DEFAULT_LOOPS = 2;
constexpr uint8_t PATTERN_MARGIN_PERCENT = 8;

HardwareSerial &tmcSerial = Serial1;
TMC2209Stepper xDriver(&tmcSerial, R_SENSE_OHMS, X_TMC_ADDRESS);
TMC2209Stepper yDriver(&tmcSerial, R_SENSE_OHMS, Y_TMC_ADDRESS);
FastAccelStepperEngine engine;
FastAccelStepper *xStepper = NULL;
FastAccelStepper *yStepper = NULL;

Axis xAxis = {"X", 0, 0};
Axis yAxis = {"Y", 0, 0};
MotorAdapter xMotor(xAxis, xDriver, xStepper, X_STEP_PIN, X_DIR_PIN);
MotorAdapter yMotor(yAxis, yDriver, yStepper, Y_STEP_PIN, Y_DIR_PIN);
MotorAdapter *activeMotor = &xMotor;

StallDetector stallDetector(STALL_SAMPLE_INTERVAL_MS, STALL_CONFIRM_SAMPLES, SEEK_SETTLE_STEPS);
bool rangeTestActive = false;
uint8_t rangeTestLegsRemaining = 0;
bool rangeTestMoveTowardMax = false;
MotorAdapter *rangeTestMotor = nullptr;
bool patternTestActive = false;

void printDriverSample();
void stopPatternTest(const __FlashStringHelper *reason);

const HomingStateMachineConfig homingConfig = {
    ARMING_STEPS,
    MAX_SEEK_STEPS,
    BACKOFF_STEPS,
    DIAGNOSTIC_INTERVAL_MS,
    static_cast<int32_t>(X_AXIS_RANGE_FULLSTEP *MICROSTEPS),
    static_cast<int32_t>(Y_AXIS_RANGE_FULLSTEP *MICROSTEPS),
    HOMING_SPEED_HZ,
    HOMING_ACCELERATION,
};

HomingStateMachine homing(
    homingConfig,
    activeMotor,
    xMotor,
    yMotor,
    stallDetector,
    rangeTestActive,
    rangeTestMotor,
    patternTestActive,
    printDriverSample,
    stopPatternTest);

enum class PatternKind
{
    Square,
    Diamond,
    Figure8,
    Spiral,
};

struct PatternWaypoint
{
    uint8_t xPercent;
    uint8_t yPercent;
    uint16_t dwellMs;
};

PatternKind activePattern = PatternKind::Square;
uint8_t patternWaypointIndex = 0;
uint8_t patternLoopsRemaining = 0;
uint32_t patternDwellStartMs = 0;
bool patternDwellActive = false;

const PatternWaypoint squarePattern[] = {
    {0, 0, 120},
    {100, 0, 120},
    {100, 100, 120},
    {0, 100, 120},
};

const PatternWaypoint diamondPattern[] = {
    {50, 0, 120},
    {100, 50, 120},
    {50, 100, 120},
    {0, 50, 120},
};

const PatternWaypoint figure8Pattern[] = {
    {20, 50, 0},
    {35, 30, 0},
    {50, 20, 0},
    {65, 30, 0},
    {80, 50, 0},
    {65, 70, 0},
    {50, 80, 0},
    {35, 70, 0},
    {20, 50, 80},
    {35, 70, 0},
    {50, 60, 0},
    {65, 70, 0},
    {80, 50, 0},
    {65, 30, 0},
    {50, 40, 0},
    {35, 30, 0},
};

const PatternWaypoint spiralPattern[] = {
    {50, 50, 0},
    {60, 50, 0},
    {60, 60, 0},
    {40, 60, 0},
    {40, 40, 0},
    {70, 40, 0},
    {70, 70, 0},
    {30, 70, 0},
    {30, 30, 0},
    {80, 30, 0},
    {80, 80, 0},
    {20, 80, 0},
    {20, 20, 0},
    {50, 20, 0},
    {50, 50, 160},
};

void startNextRangeTestLeg();
void printPatternHelp();
void updatePatternTest();

const char *patternName(PatternKind pattern)
{
    switch (pattern)
    {
    case PatternKind::Square:
        return "Square";
    case PatternKind::Diamond:
        return "Diamond";
    case PatternKind::Figure8:
        return "Figure-8";
    case PatternKind::Spiral:
        return "Spiral";
    }

    return "Unknown";
}

void patternData(PatternKind pattern, const PatternWaypoint *&waypoints, uint8_t &count)
{
    switch (pattern)
    {
    case PatternKind::Square:
        waypoints = squarePattern;
        count = static_cast<uint8_t>(sizeof(squarePattern) / sizeof(squarePattern[0]));
        return;
    case PatternKind::Diamond:
        waypoints = diamondPattern;
        count = static_cast<uint8_t>(sizeof(diamondPattern) / sizeof(diamondPattern[0]));
        return;
    case PatternKind::Figure8:
        waypoints = figure8Pattern;
        count = static_cast<uint8_t>(sizeof(figure8Pattern) / sizeof(figure8Pattern[0]));
        return;
    case PatternKind::Spiral:
        waypoints = spiralPattern;
        count = static_cast<uint8_t>(sizeof(spiralPattern) / sizeof(spiralPattern[0]));
        return;
    }

    waypoints = squarePattern;
    count = static_cast<uint8_t>(sizeof(squarePattern) / sizeof(squarePattern[0]));
}

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

void disableAllAxes()
{
    xMotor.disableOutputs();
    yMotor.disableOutputs();
}

bool startMoveTo(int32_t position)
{
    if (!activeMotor->moveTo(position))
    {
        Serial.println(F("Range test fault: FastAccelStepper rejected the moveTo command."));
        return false;
    }

    return true;
}

bool axisRangeIsKnown(const Axis &axis)
{
    return axis.axisRangeSteps > 0;
}

bool allAxisRangesKnown()
{
    return axisRangeIsKnown(xAxis) && axisRangeIsKnown(yAxis);
}

int32_t axisPositionFromPercent(const Axis &axis, uint8_t percent)
{
    const int32_t axisRange = axis.axisRangeSteps;
    if (axisRange <= 0)
    {
        return 0;
    }

    int32_t marginSteps = static_cast<int32_t>((static_cast<uint32_t>(axisRange) * PATTERN_MARGIN_PERCENT) / 100U);
    if (marginSteps < 0)
    {
        marginSteps = 0;
    }

    int32_t minPos = marginSteps;
    int32_t maxPos = axisRange - marginSteps;
    if (maxPos < minPos)
    {
        minPos = 0;
        maxPos = axisRange;
    }

    const int32_t travel = maxPos - minPos;
    return minPos + static_cast<int32_t>((static_cast<int64_t>(travel) * percent) / 100);
}

bool configurePatternMotionProfile()
{
    if (!xMotor.isInitialized() || !yMotor.isInitialized())
    {
        Serial.println(F("Pattern refused: steppers are not initialized."));
        return false;
    }

    if (!xMotor.setProfile(PATTERN_TEST_SPEED_HZ, PATTERN_TEST_ACCELERATION) ||
        !yMotor.setProfile(PATTERN_TEST_SPEED_HZ, PATTERN_TEST_ACCELERATION))
    {
        Serial.println(F("Pattern refused: invalid pattern speed or acceleration."));
        return false;
    }

    return true;
}

void restoreDefaultMotionProfile()
{
    xMotor.setProfile(HOMING_SPEED_HZ, HOMING_ACCELERATION);
    yMotor.setProfile(HOMING_SPEED_HZ, HOMING_ACCELERATION);
}

void stopPatternTest(const __FlashStringHelper *reason)
{
    if (!patternTestActive)
    {
        return;
    }

    patternTestActive = false;
    patternDwellActive = false;
    restoreDefaultMotionProfile();
    disableAllAxes();
    Serial.print(F("Pattern stopped: "));
    Serial.println(reason);
}

bool startPatternMoveToWaypoint(uint8_t index)
{
    const PatternWaypoint *waypoints = nullptr;
    uint8_t waypointCount = 0;
    patternData(activePattern, waypoints, waypointCount);
    if (index >= waypointCount)
    {
        return false;
    }

    const int32_t targetX = axisPositionFromPercent(xAxis, waypoints[index].xPercent);
    const int32_t targetY = axisPositionFromPercent(yAxis, waypoints[index].yPercent);

    if (!xMotor.moveTo(targetX))
    {
        Serial.println(F("Pattern fault: X moveTo rejected."));
        stopPatternTest(F("X move rejected"));
        return false;
    }

    if (!yMotor.moveTo(targetY))
    {
        Serial.println(F("Pattern fault: Y moveTo rejected."));
        stopPatternTest(F("Y move rejected"));
        return false;
    }

    Serial.print(F("Pattern "));
    Serial.print(patternName(activePattern));
    Serial.print(F(": waypoint "));
    Serial.print(index + 1);
    Serial.print(F(" x="));
    Serial.print(targetX);
    Serial.print(F(" y="));
    Serial.println(targetY);
    return true;
}

void beginPatternTest(PatternKind pattern)
{
    if (rangeTestActive)
    {
        Serial.println(F("Pattern refused: range test is active."));
        return;
    }

    if (patternTestActive)
    {
        Serial.println(F("Pattern is already active."));
        return;
    }

    if (!homing.isIdle())
    {
        Serial.println(F("Pattern refused: homing is active or faulted."));
        return;
    }

    if (!allAxisRangesKnown())
    {
        Serial.println(F("Pattern refused: run homing first so both axis ranges are known."));
        return;
    }

    if (!configurePatternMotionProfile())
    {
        return;
    }

    activePattern = pattern;
    patternWaypointIndex = 0;
    patternLoopsRemaining = (pattern == PatternKind::Spiral) ? 1 : PATTERN_TEST_DEFAULT_LOOPS;
    patternDwellStartMs = 0;
    patternDwellActive = false;
    patternTestActive = true;

    Serial.print(F("Pattern start: "));
    Serial.print(patternName(activePattern));
    Serial.print(F(" loops="));
    Serial.println(patternLoopsRemaining);

    if (!startPatternMoveToWaypoint(patternWaypointIndex))
    {
        stopPatternTest(F("invalid waypoint"));
    }
}

int32_t safeMinPosition()
{
    return 0;
}

int32_t safeMaxPosition()
{
    return activeMotor->axisState().axisRangeSteps;
}

bool startRangeTestForMotor(MotorAdapter &motor)
{
    activeMotor = &motor;
    rangeTestMotor = &motor;

    if (!activeMotor->setProfile(RANGE_TEST_SPEED_HZ, RANGE_TEST_ACCELERATION))
    {
        Serial.print(F("Range test refused for axis "));
        Serial.print(motor.axisName());
        Serial.println(F(": invalid speed or acceleration."));
        return false;
    }

    rangeTestLegsRemaining = RANGE_TEST_CYCLES * 2;
    rangeTestMoveTowardMax = true;

    Serial.print(F("Range test axis "));
    Serial.print(motor.axisName());
    Serial.print(F(": cycling "));
    Serial.print(RANGE_TEST_CYCLES);
    Serial.print(F(" times between "));
    Serial.print(safeMinPosition());
    Serial.print(F(" and "));
    Serial.println(safeMaxPosition());
    startNextRangeTestLeg();
    return true;
}

void finishRangeTest()
{
    activeMotor->setProfile(HOMING_SPEED_HZ, HOMING_ACCELERATION);
    disableAxis();
    Serial.print(F("Range test axis "));
    Serial.print(activeMotor->axisName());
    Serial.print(F(" complete. Final position="));
    Serial.println(activeMotor->position());

    if (rangeTestMotor == &xMotor)
    {
        Serial.println(F("Range test: starting Y axis."));
        if (startRangeTestForMotor(yMotor))
        {
            return;
        }
    }

    rangeTestActive = false;
    rangeTestMotor = nullptr;
    activeMotor = &xMotor;
    Serial.println(F("Range test complete on X and Y."));
}

void startNextRangeTestLeg()
{
    if (rangeTestLegsRemaining == 0)
    {
        finishRangeTest();
        return;
    }

    const int32_t targetPosition = rangeTestMoveTowardMax ? safeMaxPosition() : safeMinPosition();
    Serial.print(F("Range test axis "));
    Serial.print(activeMotor->axisName());
    Serial.print(F(": moving to "));
    Serial.print(targetPosition);
    Serial.print(F(" at speed="));
    Serial.print(RANGE_TEST_SPEED_HZ);
    Serial.print(F(" remaining_legs="));
    Serial.println(rangeTestLegsRemaining);

    if (!startMoveTo(targetPosition))
    {
        finishRangeTest();
        return;
    }

    rangeTestMoveTowardMax = !rangeTestMoveTowardMax;
    rangeTestLegsRemaining--;
}

void beginRangeTest()
{
    if (rangeTestActive)
    {
        Serial.println(F("Range test is already active."));
        return;
    }

    if (!homing.isIdle())
    {
        Serial.println(F("Range test refused: homing is active or faulted."));
        return;
    }

    if (patternTestActive)
    {
        Serial.println(F("Range test refused: pattern test is active."));
        return;
    }

    if (!allAxisRangesKnown())
    {
        Serial.println(F("Range test refused: run homing first so both axis ranges are known."));
        return;
    }

    rangeTestActive = true;
    if (!startRangeTestForMotor(xMotor))
    {
        rangeTestActive = false;
        rangeTestMotor = nullptr;
        activeMotor = &xMotor;
    }
}

void updateRangeTest()
{
    if (!rangeTestActive || activeMotor->isRunning())
    {
        return;
    }

    Serial.print(F("Range test axis "));
    Serial.print(activeMotor->axisName());
    Serial.print(F(": reached position="));
    Serial.println(activeMotor->position());
    startNextRangeTestLeg();
}

void updatePatternTest()
{
    if (!patternTestActive)
    {
        return;
    }

    if (xMotor.isRunning() || yMotor.isRunning())
    {
        return;
    }

    const PatternWaypoint *waypoints = nullptr;
    uint8_t waypointCount = 0;
    patternData(activePattern, waypoints, waypointCount);
    if (waypointCount == 0)
    {
        stopPatternTest(F("no waypoints"));
        return;
    }

    const PatternWaypoint &currentWaypoint = waypoints[patternWaypointIndex];
    if (currentWaypoint.dwellMs > 0)
    {
        if (!patternDwellActive)
        {
            patternDwellActive = true;
            patternDwellStartMs = millis();
            return;
        }

        if (millis() - patternDwellStartMs < currentWaypoint.dwellMs)
        {
            return;
        }
    }

    patternDwellActive = false;
    patternWaypointIndex++;

    if (patternWaypointIndex >= waypointCount)
    {
        if (patternLoopsRemaining > 1)
        {
            patternLoopsRemaining--;
            patternWaypointIndex = 0;
            Serial.print(F("Pattern "));
            Serial.print(patternName(activePattern));
            Serial.print(F(": next loop, remaining="));
            Serial.println(patternLoopsRemaining);
        }
        else
        {
            stopPatternTest(F("completed"));
            return;
        }
    }

    startPatternMoveToWaypoint(patternWaypointIndex);
}

void printPatternHelp()
{
    Serial.println(F("Pattern commands:"));
    Serial.println(F("  c = axis range sweep (existing test)"));
    Serial.println(F("  1 = square"));
    Serial.println(F("  2 = diamond"));
    Serial.println(F("  3 = figure-8"));
    Serial.println(F("  4 = spiral"));
    Serial.println(F("  x = abort active homing/range/pattern"));
}

bool setupDrivers()
{
    tmcSerial.begin(TMC_UART_BAUD, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
    const bool configurationComplete = configurationIsComplete();
    return xMotor.configureDriver(MOTOR_RMS_CURRENT_MA, MICROSTEPS, STALLGUARD_TCOOLTHRS, STALLGUARD_SGTHRS, configurationComplete) &&
           yMotor.configureDriver(MOTOR_RMS_CURRENT_MA, MICROSTEPS, STALLGUARD_TCOOLTHRS, STALLGUARD_SGTHRS, configurationComplete);
}

bool setupMotion()
{
    engine.init();
    return xMotor.setupMotion(engine, ENN_PIN, HOMING_SPEED_HZ, HOMING_ACCELERATION) &&
           yMotor.setupMotion(engine, ENN_PIN, HOMING_SPEED_HZ, HOMING_ACCELERATION);
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
    Serial.println(F("Ready. Send 's' to home, 'c' for range test, '1'-'4' for patterns, 'x' to abort, or 'd' for StallGuard diagnostics."));
    printPatternHelp();
}

void loop()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());
        if (command == 's' || command == 'S')
        {
            homing.begin(configurationIsComplete());
        }
        else if (command == 'x' || command == 'X')
        {
            homing.abort(F("serial abort"));
        }
        else if (command == 'd' || command == 'D')
        {
            printDriverSample();
        }
        else if (command == 'c' || command == 'C')
        {
            beginRangeTest();
        }
        else if (command == '1')
        {
            beginPatternTest(PatternKind::Square);
        }
        else if (command == '2')
        {
            beginPatternTest(PatternKind::Diamond);
        }
        else if (command == '3')
        {
            beginPatternTest(PatternKind::Figure8);
        }
        else if (command == '4')
        {
            beginPatternTest(PatternKind::Spiral);
        }
        else if (command == 'p' || command == 'P')
        {
            printPatternHelp();
        }
    }

    homing.update();
    updateRangeTest();
    updatePatternTest();
}
