#include "TestController.h"

bool TestController::isActive() const
{
    return rangeTestActive || patternTestActive;
}

void TestController::beginRangeTest()
{
    if (rangeTestActive)
    {
        Serial.println(F("Range test is already active."));
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

void TestController::beginPatternTest(PatternKind pattern)
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
    patternLoopsRemaining = (pattern == PatternKind::Spiral) ? 1 : config.patternTestDefaultLoops;
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

void TestController::cancel(const __FlashStringHelper *reason)
{
    if (patternTestActive)
    {
        if (xMotor.isRunning())
        {
            xMotor.forceStop();
        }
        if (yMotor.isRunning())
        {
            yMotor.forceStop();
        }
        stopPatternTest(reason);
        return;
    }

    if (rangeTestActive)
    {
        activeMotor->forceStop();
        rangeTestActive = false;
        rangeTestMotor = nullptr;
        activeMotor->setProfile(config.homingSpeedHz, config.homingAcceleration);
        activeMotor->disableOutputs();
        activeMotor = &xMotor;
        Serial.print(F("Range test aborted: "));
        Serial.println(reason);
    }
}

void TestController::update()
{
    updateRangeTest();
    updatePatternTest();
}

bool TestController::allAxisRangesKnown() const
{
    return axisRangeIsKnown(xMotor.axisState()) && axisRangeIsKnown(yMotor.axisState());
}

bool TestController::axisRangeIsKnown(const Axis &axis) const
{
    return axis.axisRangeSteps > 0;
}

int32_t TestController::axisPositionFromPercent(const Axis &axis, uint8_t percent) const
{
    const int32_t axisRange = axis.axisRangeSteps;
    if (axisRange <= 0)
    {
        return 0;
    }

    int32_t marginSteps = static_cast<int32_t>((static_cast<uint32_t>(axisRange) * config.patternMarginPercent) / 100U);
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

bool TestController::configurePatternMotionProfile()
{
    if (!xMotor.isInitialized() || !yMotor.isInitialized())
    {
        Serial.println(F("Pattern refused: steppers are not initialized."));
        return false;
    }

    if (!xMotor.setProfile(config.patternTestSpeedHz, config.patternTestAcceleration) ||
        !yMotor.setProfile(config.patternTestSpeedHz, config.patternTestAcceleration))
    {
        Serial.println(F("Pattern refused: invalid pattern speed or acceleration."));
        return false;
    }

    return true;
}

void TestController::restoreDefaultMotionProfile()
{
    xMotor.setProfile(config.homingSpeedHz, config.homingAcceleration);
    yMotor.setProfile(config.homingSpeedHz, config.homingAcceleration);
}

void TestController::disableAllAxes()
{
    xMotor.disableOutputs();
    yMotor.disableOutputs();
}

void TestController::stopPatternTest(const __FlashStringHelper *reason)
{
    if (!patternTestActive)
    {
        return;
    }

    if (xMotor.isRunning())
    {
        xMotor.forceStop();
    }
    if (yMotor.isRunning())
    {
        yMotor.forceStop();
    }

    patternTestActive = false;
    patternDwellActive = false;
    restoreDefaultMotionProfile();
    disableAllAxes();
    Serial.print(F("Pattern stopped: "));
    Serial.println(reason);
}

bool TestController::startPatternMoveToWaypoint(uint8_t index)
{
    const PatternWaypoint *waypoints = nullptr;
    uint8_t waypointCount = 0;
    patternData(activePattern, waypoints, waypointCount);
    if (index >= waypointCount)
    {
        return false;
    }

    const int32_t targetX = axisPositionFromPercent(xMotor.axisState(), waypoints[index].xPercent);
    const int32_t targetY = axisPositionFromPercent(yMotor.axisState(), waypoints[index].yPercent);

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

bool TestController::startRangeTestForMotor(MotorAdapter &motor)
{
    activeMotor = &motor;
    rangeTestMotor = &motor;

    if (!activeMotor->setProfile(config.rangeTestSpeedHz, config.rangeTestAcceleration))
    {
        Serial.print(F("Range test refused for axis "));
        Serial.print(motor.axisName());
        Serial.println(F(": invalid speed or acceleration."));
        return false;
    }

    rangeTestLegsRemaining = config.rangeTestCycles * 2;
    rangeTestMoveTowardMax = true;

    Serial.print(F("Range test axis "));
    Serial.print(motor.axisName());
    Serial.print(F(": cycling "));
    Serial.print(config.rangeTestCycles);
    Serial.print(F(" times between "));
    Serial.print(safeMinPosition());
    Serial.print(F(" and "));
    Serial.println(safeMaxPosition());
    startNextRangeTestLeg();
    return true;
}

void TestController::finishRangeTest()
{
    activeMotor->setProfile(config.homingSpeedHz, config.homingAcceleration);
    activeMotor->disableOutputs();
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

void TestController::startNextRangeTestLeg()
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
    Serial.print(config.rangeTestSpeedHz);
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

void TestController::updateRangeTest()
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

void TestController::updatePatternTest()
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

bool TestController::startMoveTo(int32_t position)
{
    if (!activeMotor->moveTo(position))
    {
        Serial.println(F("Range test fault: FastAccelStepper rejected the moveTo command."));
        return false;
    }

    return true;
}

int32_t TestController::safeMinPosition() const
{
    return 0;
}

int32_t TestController::safeMaxPosition() const
{
    return activeMotor->axisState().axisRangeSteps;
}

const char *TestController::patternName(PatternKind pattern) const
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

void TestController::patternData(PatternKind pattern, const PatternWaypoint *&waypoints, uint8_t &count) const
{
    static const PatternWaypoint squarePattern[] = {
        {0, 0, 120},
        {100, 0, 120},
        {100, 100, 120},
        {0, 100, 120},
    };

    static const PatternWaypoint diamondPattern[] = {
        {50, 0, 120},
        {100, 50, 120},
        {50, 100, 120},
        {0, 50, 120},
    };

    static const PatternWaypoint figure8Pattern[] = {
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

    static const PatternWaypoint spiralPattern[] = {
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