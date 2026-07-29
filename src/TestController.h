#pragma once

#include <Arduino.h>

#include "MotorAdapter.h"

enum class PatternKind
{
    Square,
    Diamond,
    Figure8,
    Spiral,
};

struct TestControllerConfig
{
    uint32_t homingSpeedHz;
    uint32_t homingAcceleration;
    uint32_t rangeTestSpeedHz;
    uint32_t rangeTestAcceleration;
    uint8_t rangeTestCycles;
    uint32_t patternTestSpeedHz;
    uint32_t patternTestAcceleration;
    uint8_t patternTestDefaultLoops;
    uint8_t patternMarginPercent;
};

class TestController
{
public:
    TestController(
        const TestControllerConfig &config,
        MotorAdapter *&activeMotor,
        MotorAdapter &xMotor,
        MotorAdapter &yMotor)
        : config(config), activeMotor(activeMotor), xMotor(xMotor), yMotor(yMotor)
    {
    }

    bool isActive() const;
    void beginRangeTest();
    void beginPatternTest(PatternKind pattern);
    void cancel(const __FlashStringHelper *reason);
    void update();
    void printPatternHelp() const;

private:
    struct PatternWaypoint
    {
        uint8_t xPercent;
        uint8_t yPercent;
        uint16_t dwellMs;
    };

    const TestControllerConfig &config;
    MotorAdapter *&activeMotor;
    MotorAdapter &xMotor;
    MotorAdapter &yMotor;

    bool rangeTestActive = false;
    uint8_t rangeTestLegsRemaining = 0;
    bool rangeTestMoveTowardMax = false;
    MotorAdapter *rangeTestMotor = nullptr;

    bool patternTestActive = false;
    PatternKind activePattern = PatternKind::Square;
    uint8_t patternWaypointIndex = 0;
    uint8_t patternLoopsRemaining = 0;
    uint32_t patternDwellStartMs = 0;
    bool patternDwellActive = false;

    bool allAxisRangesKnown() const;
    bool axisRangeIsKnown(const Axis &axis) const;
    int32_t axisPositionFromPercent(const Axis &axis, uint8_t percent) const;
    bool configurePatternMotionProfile();
    void restoreDefaultMotionProfile();
    void disableAllAxes();
    void stopPatternTest(const __FlashStringHelper *reason);
    bool startPatternMoveToWaypoint(uint8_t index);
    bool startRangeTestForMotor(MotorAdapter &motor);
    void finishRangeTest();
    void startNextRangeTestLeg();
    void updateRangeTest();
    void updatePatternTest();
    bool startMoveTo(int32_t position);
    int32_t safeMinPosition() const;
    int32_t safeMaxPosition() const;
    const char *patternName(PatternKind pattern) const;
    void patternData(PatternKind pattern, const PatternWaypoint *&waypoints, uint8_t &count) const;
};