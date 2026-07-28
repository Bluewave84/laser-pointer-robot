#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>

// TMC2209 sensorless homing for UM FeatherS2.
// This sketch uses a low-to-high DIAG transition after an arming move, not a
// continuously high DIAG level, so a stale DIAG signal cannot home the axis.

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
constexpr uint8_t DIAG_PIN = 10;

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

struct Axis
{
    const char *name;
    TMC2209Stepper *driver;
    FastAccelStepper **stepper;
    uint8_t stepPin;
    uint8_t dirPin;
    int32_t physicalAxisRangeSteps;
    int32_t axisRangeSteps;
};

Axis xAxis = {"X", &xDriver, &xStepper, X_STEP_PIN, X_DIR_PIN, 0, 0};
Axis yAxis = {"Y", &yDriver, &yStepper, Y_STEP_PIN, Y_DIR_PIN, 0, 0};
Axis *activeAxis = &xAxis;

enum class HomingState
{
    Idle,
    Arming,
    Seeking,
    StoppingAtStall,
    BackingOff,
    MovingToMinimum,
    Fault,
};

enum class HomingPhase
{
    FindZero,
    FindMax,
};

HomingState homingState = HomingState::Idle;
HomingPhase homingPhase = HomingPhase::FindZero;
volatile bool diagRisingEdgeSeen = false;
uint32_t lastDiagnosticMs = 0;
uint32_t lastStallSampleMs = 0;
uint8_t stallConfirmSamples = 0;
int32_t seekStartPosition = 0;
bool rangeTestActive = false;
uint8_t rangeTestLegsRemaining = 0;
bool rangeTestMoveTowardMax = false;
Axis *rangeTestAxis = nullptr;

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

bool patternTestActive = false;
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

FastAccelStepper *currentStepper()
{
    return *activeAxis->stepper;
}

TMC2209Stepper &currentDriver()
{
    return *activeAxis->driver;
}

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

void IRAM_ATTR onDiagRising()
{
    diagRisingEdgeSeen = true;
}

bool configurationIsComplete()
{
    return R_SENSE_OHMS > 0.0f && MOTOR_RMS_CURRENT_MA > 0 &&
           STALLGUARD_TCOOLTHRS > 0 && STALLGUARD_SGTHRS > 0 &&
           MAX_SEEK_STEPS > ARMING_STEPS && BACKOFF_STEPS > 0;
}

void clearDiagEdge()
{
    noInterrupts();
    diagRisingEdgeSeen = false;
    interrupts();
}

bool consumeDiagEdge()
{
    noInterrupts();
    const bool edgeSeen = diagRisingEdgeSeen;
    diagRisingEdgeSeen = false;
    interrupts();
    return edgeSeen;
}

void resetStallConfirmation()
{
    lastStallSampleMs = 0;
    stallConfirmSamples = 0;
}

bool stallConfirmedBySgResult()
{
    if (abs(currentStepper()->getCurrentPosition() - seekStartPosition) < SEEK_SETTLE_STEPS)
    {
        resetStallConfirmation();
        return false;
    }

    if (millis() - lastStallSampleMs < STALL_SAMPLE_INTERVAL_MS)
    {
        return false;
    }

    lastStallSampleMs = millis();
    const uint16_t sgResult = currentDriver().SG_RESULT();
    if (sgResult <= currentDriver().SGTHRS())
    {
        stallConfirmSamples++;
    }
    else
    {
        stallConfirmSamples = 0;
    }

    return stallConfirmSamples >= STALL_CONFIRM_SAMPLES;
}

void printDriverSample()
{
    Serial.print(F("[SG] diag="));
    Serial.print(digitalRead(DIAG_PIN));
    Serial.print(F(" axis="));
    Serial.print(activeAxis->name);
    Serial.print(F(" sg_result="));
    Serial.print(currentDriver().SG_RESULT());
    Serial.print(F(" sgthrs="));
    Serial.println(currentDriver().SGTHRS());
}

void printMotionSample()
{
    if (millis() - lastDiagnosticMs < DIAGNOSTIC_INTERVAL_MS)
    {
        return;
    }

    lastDiagnosticMs = millis();
    Serial.print(F("[HOME] position="));
    Serial.print(currentStepper()->getCurrentPosition());
    Serial.print(F(" axis="));
    Serial.print(activeAxis->name);
    Serial.print(F(" state="));
    Serial.print(static_cast<uint8_t>(homingState));
    Serial.print(' ');
    printDriverSample();
}

void disableAxis()
{
    if (currentStepper() != nullptr)
    {
        currentStepper()->disableOutputs();
    }
}

void disableAllAxes()
{
    if (xStepper != nullptr)
    {
        xStepper->disableOutputs();
    }

    if (yStepper != nullptr)
    {
        yStepper->disableOutputs();
    }
}

void failHoming(const __FlashStringHelper *reason)
{
    if (currentStepper() != nullptr && currentStepper()->isRunning())
    {
        currentStepper()->forceStop();
    }

    homingState = HomingState::Fault;
    Serial.print(F("Homing fault: "));
    Serial.println(reason);
    printDriverSample();
}

bool startMove(int32_t steps)
{
    if (currentStepper()->move(steps) != MOVE_OK)
    {
        failHoming(F("FastAccelStepper rejected the move"));
        return false;
    }

    return true;
}

bool startMoveTo(int32_t position)
{
    if (currentStepper()->moveTo(position) != MOVE_OK)
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
    if (xStepper == nullptr || yStepper == nullptr)
    {
        Serial.println(F("Pattern refused: steppers are not initialized."));
        return false;
    }

    if (xStepper->setSpeedInHz(PATTERN_TEST_SPEED_HZ) != 0 ||
        xStepper->setAcceleration(PATTERN_TEST_ACCELERATION) != 0 ||
        yStepper->setSpeedInHz(PATTERN_TEST_SPEED_HZ) != 0 ||
        yStepper->setAcceleration(PATTERN_TEST_ACCELERATION) != 0)
    {
        Serial.println(F("Pattern refused: invalid pattern speed or acceleration."));
        return false;
    }

    return true;
}

void restoreDefaultMotionProfile()
{
    if (xStepper != nullptr)
    {
        xStepper->setSpeedInHz(HOMING_SPEED_HZ);
        xStepper->setAcceleration(HOMING_ACCELERATION);
    }

    if (yStepper != nullptr)
    {
        yStepper->setSpeedInHz(HOMING_SPEED_HZ);
        yStepper->setAcceleration(HOMING_ACCELERATION);
    }
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

    if (xStepper->moveTo(targetX) != MOVE_OK)
    {
        Serial.println(F("Pattern fault: X moveTo rejected."));
        stopPatternTest(F("X move rejected"));
        return false;
    }

    if (yStepper->moveTo(targetY) != MOVE_OK)
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

    if (homingState != HomingState::Idle)
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

int32_t configuredAxisRangeSteps(const Axis &axis)
{
    const uint32_t configuredFullSteps = (&axis == &xAxis) ? X_AXIS_RANGE_FULLSTEP : Y_AXIS_RANGE_FULLSTEP;
    if (configuredFullSteps == 0)
    {
        return 0;
    }

    return static_cast<int32_t>(configuredFullSteps * MICROSTEPS);
}

void finishCurrentAxisHoming()
{
    disableAxis();
    homingState = HomingState::Idle;
    Serial.print(F("Homing complete. Range 0.."));
    Serial.print(activeAxis->axisRangeSteps);
    Serial.print(F(" steps, final position="));
    Serial.println(currentStepper()->getCurrentPosition());

    if (activeAxis == &xAxis)
    {
        activeAxis = &yAxis;
        clearDiagEdge();
        resetStallConfirmation();
        lastDiagnosticMs = 0;
        homingPhase = HomingPhase::FindZero;
        homingState = HomingState::Arming;
        Serial.println(F("Homing: starting Y axis."));
        Serial.println(F("Homing: arming before seeking zero end."));
        startMove(ARMING_STEPS);
    }
}

int32_t safeMinPosition()
{
    return 0;
}

int32_t safeMaxPosition()
{
    return activeAxis->axisRangeSteps;
}

int32_t seekStepsForCurrentPhase()
{
    return homingPhase == HomingPhase::FindZero ? -MAX_SEEK_STEPS : MAX_SEEK_STEPS;
}

int32_t backoffStepsForCurrentPhase()
{
    return homingPhase == HomingPhase::FindZero ? BACKOFF_STEPS : -BACKOFF_STEPS;
}

void startSeekingCurrentPhase()
{
    clearDiagEdge();
    resetStallConfirmation();
    seekStartPosition = currentStepper()->getCurrentPosition();
    homingState = HomingState::Seeking;

    if (homingPhase == HomingPhase::FindZero)
    {
        Serial.print(F("Homing: seeking zero end using SG_RESULT threshold. "));
    }
    else
    {
        Serial.print(F("Homing: seeking max end using SG_RESULT threshold. "));
    }

    printDriverSample();
    startMove(seekStepsForCurrentPhase());
}

void beginHoming()
{
    if (rangeTestActive)
    {
        Serial.println(F("Homing refused: range test is active."));
        return;
    }

    if (patternTestActive)
    {
        Serial.println(F("Homing refused: a pattern test is active."));
        return;
    }

    if (homingState != HomingState::Idle && homingState != HomingState::Fault)
    {
        Serial.println(F("Homing is already active."));
        return;
    }

    if (!configurationIsComplete())
    {
        Serial.println(F("Set R_SENSE_OHMS, MOTOR_RMS_CURRENT_MA, STALLGUARD_TCOOLTHRS, and STALLGUARD_SGTHRS before homing."));
        return;
    }

    if (digitalRead(DIAG_PIN) == HIGH)
    {
        Serial.println(F("Homing warning: DIAG is already high before motion; using SG_RESULT instead."));
        printDriverSample();
    }

    clearDiagEdge();
    resetStallConfirmation();
    lastDiagnosticMs = 0;
    xAxis.physicalAxisRangeSteps = 0;
    xAxis.axisRangeSteps = 0;
    yAxis.physicalAxisRangeSteps = 0;
    yAxis.axisRangeSteps = 0;

    xAxis.axisRangeSteps = configuredAxisRangeSteps(xAxis);
    if (xAxis.axisRangeSteps > 0)
    {
        Serial.print(F("Homing: X configured range set to 0.."));
        Serial.print(xAxis.axisRangeSteps);
        Serial.println(F(" steps (FindMax will be skipped)."));
    }
    else
    {
        Serial.println(F("Homing: X range unknown; full FindZero + FindMax homing will run."));
    }

    yAxis.axisRangeSteps = configuredAxisRangeSteps(yAxis);
    if (yAxis.axisRangeSteps > 0)
    {
        Serial.print(F("Homing: Y configured range set to 0.."));
        Serial.print(yAxis.axisRangeSteps);
        Serial.println(F(" steps (FindMax will be skipped)."));
    }
    else
    {
        Serial.println(F("Homing: Y range unknown; full FindZero + FindMax homing will run."));
    }

    activeAxis = &xAxis;
    homingPhase = HomingPhase::FindZero;
    homingState = HomingState::Arming;
    Serial.println(F("Homing: starting X axis."));
    Serial.println(F("Homing: arming before seeking zero end."));
    startMove(ARMING_STEPS);
}

void abortHoming(const __FlashStringHelper *reason)
{
    if (patternTestActive)
    {
        if (xStepper != nullptr && xStepper->isRunning())
        {
            xStepper->forceStop();
        }
        if (yStepper != nullptr && yStepper->isRunning())
        {
            yStepper->forceStop();
        }
        stopPatternTest(reason);
        return;
    }

    if (rangeTestActive)
    {
        currentStepper()->forceStop();
        rangeTestActive = false;
        rangeTestAxis = nullptr;
        currentStepper()->setSpeedInHz(HOMING_SPEED_HZ);
        currentStepper()->setAcceleration(HOMING_ACCELERATION);
        activeAxis = &xAxis;
        disableAxis();
        Serial.print(F("Range test aborted: "));
        Serial.println(reason);
        return;
    }

    failHoming(reason);
}

bool startRangeTestForAxis(Axis &axis)
{
    activeAxis = &axis;
    rangeTestAxis = &axis;

    if (currentStepper()->setSpeedInHz(RANGE_TEST_SPEED_HZ) != 0 ||
        currentStepper()->setAcceleration(RANGE_TEST_ACCELERATION) != 0)
    {
        Serial.print(F("Range test refused for axis "));
        Serial.print(axis.name);
        Serial.println(F(": invalid speed or acceleration."));
        return false;
    }

    rangeTestLegsRemaining = RANGE_TEST_CYCLES * 2;
    rangeTestMoveTowardMax = true;

    Serial.print(F("Range test axis "));
    Serial.print(axis.name);
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
    currentStepper()->setSpeedInHz(HOMING_SPEED_HZ);
    currentStepper()->setAcceleration(HOMING_ACCELERATION);
    disableAxis();
    Serial.print(F("Range test axis "));
    Serial.print(activeAxis->name);
    Serial.print(F(" complete. Final position="));
    Serial.println(currentStepper()->getCurrentPosition());

    if (rangeTestAxis == &xAxis)
    {
        Serial.println(F("Range test: starting Y axis."));
        if (startRangeTestForAxis(yAxis))
        {
            return;
        }
    }

    rangeTestActive = false;
    rangeTestAxis = nullptr;
    activeAxis = &xAxis;
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
    Serial.print(activeAxis->name);
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

    if (homingState != HomingState::Idle)
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
    if (!startRangeTestForAxis(xAxis))
    {
        rangeTestActive = false;
        rangeTestAxis = nullptr;
        activeAxis = &xAxis;
    }
}

void updateRangeTest()
{
    if (!rangeTestActive || currentStepper()->isRunning())
    {
        return;
    }

    Serial.print(F("Range test axis "));
    Serial.print(activeAxis->name);
    Serial.print(F(": reached position="));
    Serial.println(currentStepper()->getCurrentPosition());
    startNextRangeTestLeg();
}

void updatePatternTest()
{
    if (!patternTestActive)
    {
        return;
    }

    if (xStepper->isRunning() || yStepper->isRunning())
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

void updateHoming()
{
    if (homingState == HomingState::Idle)
    {
        return;
    }

    if (homingState == HomingState::Fault)
    {
        if (!currentStepper()->isRunning())
        {
            disableAxis();
        }
        return;
    }

    if (homingState == HomingState::Arming)
    {
        if (currentStepper()->isRunning())
        {
            return;
        }

        startSeekingCurrentPhase();
        return;
    }

    if (homingState == HomingState::Seeking)
    {
        if (!currentStepper()->isRunning())
        {
            failHoming(F("maximum seek travel reached without a confirmed StallGuard hit"));
            return;
        }

        consumeDiagEdge();

        if (stallConfirmedBySgResult())
        {
            Serial.println(F("Homing: StallGuard SG_RESULT threshold accepted."));
            Serial.print(F("[HOME] accepted_position="));
            Serial.println(currentStepper()->getCurrentPosition());
            printDriverSample();
            currentStepper()->forceStop();
            homingState = HomingState::StoppingAtStall;
        }

        printMotionSample();
        return;
    }

    if (homingState == HomingState::StoppingAtStall)
    {
        if (currentStepper()->isRunning())
        {
            return;
        }

        if (homingPhase == HomingPhase::FindZero)
        {
            currentStepper()->setCurrentPosition(0);
            Serial.println(F("Homing: zero end found at position 0."));
        }
        else
        {
            activeAxis->physicalAxisRangeSteps = currentStepper()->getCurrentPosition();
            Serial.print(F("Homing: max end found. Physical range steps="));
            Serial.println(activeAxis->physicalAxisRangeSteps);
        }

        clearDiagEdge();
        homingState = HomingState::BackingOff;
        Serial.println(F("Homing: backing off from the detected end stop."));
        startMove(backoffStepsForCurrentPhase());
        return;
    }

    if (homingState == HomingState::BackingOff && !currentStepper()->isRunning())
    {
        if (homingPhase == HomingPhase::FindZero)
        {
            if (axisRangeIsKnown(*activeAxis))
            {
                currentStepper()->setCurrentPosition(0);
                Serial.print(F("Homing: using configured upper limit for axis "));
                Serial.print(activeAxis->name);
                Serial.print(F(". Max position set to "));
                Serial.print(activeAxis->axisRangeSteps);
                Serial.println(F(" steps; FindMax skipped."));
                finishCurrentAxisHoming();
                return;
            }

            homingPhase = HomingPhase::FindMax;
            lastDiagnosticMs = 0;
            Serial.print(F("Homing: zero backoff position="));
            Serial.println(currentStepper()->getCurrentPosition());
            startSeekingCurrentPhase();
            return;
        }

        if (activeAxis->physicalAxisRangeSteps <= (BACKOFF_STEPS * 2))
        {
            failHoming(F("measured range is smaller than both backoff margins"));
            return;
        }

        homingState = HomingState::MovingToMinimum;
        Serial.print(F("Homing: moving to minimum usable position="));
        Serial.println(BACKOFF_STEPS);
        startMoveTo(BACKOFF_STEPS);
        return;
    }

    if (homingState == HomingState::MovingToMinimum && !currentStepper()->isRunning())
    {
        if (!axisRangeIsKnown(*activeAxis))
        {
            activeAxis->axisRangeSteps = activeAxis->physicalAxisRangeSteps - (BACKOFF_STEPS * 2);
        }
        currentStepper()->setCurrentPosition(0);

        finishCurrentAxisHoming();
    }
}

bool configureDriver(Axis &axis)
{
    axis.driver->begin();

    if (axis.driver->test_connection() != 0)
    {
        Serial.print(axis.name);
        Serial.println(F(" TMC2209 UART connection failed."));
        return false;
    }

    axis.driver->pdn_disable(true);
    axis.driver->mstep_reg_select(true);
    axis.driver->toff(4);
    axis.driver->blank_time(24);

    if (!configurationIsComplete())
    {
        Serial.print(axis.name);
        Serial.println(F(" driver connected. Configuration is incomplete; motion remains disabled."));
        return true;
    }

    axis.driver->rms_current(MOTOR_RMS_CURRENT_MA);
    axis.driver->microsteps(MICROSTEPS);
    axis.driver->en_spreadCycle(false);
    axis.driver->TCOOLTHRS(STALLGUARD_TCOOLTHRS);
    axis.driver->SGTHRS(STALLGUARD_SGTHRS);
    return true;
}

bool setupDrivers()
{
    tmcSerial.begin(TMC_UART_BAUD, SERIAL_8N1, TMC_RX_PIN, TMC_TX_PIN);
    return configureDriver(xAxis) && configureDriver(yAxis);
}

bool setupAxisMotion(Axis &axis)
{
    *axis.stepper = engine.stepperConnectToPin(axis.stepPin);
    if (*axis.stepper == nullptr)
    {
        Serial.print(axis.name);
        Serial.println(F(" FastAccelStepper initialization failed."));
        return false;
    }

    (*axis.stepper)->setDirectionPin(axis.dirPin, true, 10);
    (*axis.stepper)->setEnablePin(ENN_PIN, true);
    (*axis.stepper)->setAutoEnable(true);
    (*axis.stepper)->setDelayToEnable(2000);
    (*axis.stepper)->setDelayToDisable(100);

    if ((*axis.stepper)->setSpeedInHz(HOMING_SPEED_HZ) != 0 ||
        (*axis.stepper)->setAcceleration(HOMING_ACCELERATION) != 0)
    {
        Serial.print(axis.name);
        Serial.println(F(" invalid FastAccelStepper speed or acceleration."));
        return false;
    }

    return true;
}

bool setupMotion()
{
    engine.init();
    return setupAxisMotion(xAxis) && setupAxisMotion(yAxis);
}

void setup()
{
    Serial.begin(CONSOLE_BAUD);
    delay(500);

    pinMode(DIAG_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(DIAG_PIN), onDiagRising, RISING);

    if (!setupDrivers() || !setupMotion())
    {
        homingState = HomingState::Fault;
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
            beginHoming();
        }
        else if (command == 'x' || command == 'X')
        {
            abortHoming(F("serial abort"));
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

    updateHoming();
    updateRangeTest();
    updatePatternTest();
}
