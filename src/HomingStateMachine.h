#pragma once

#include <Arduino.h>

#include "MotorAdapter.h"
#include "StallDetector.h"
#include "TestController.h"

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

struct HomingStateMachineConfig
{
    int32_t armingSteps;
    int32_t maxSeekSteps;
    int32_t backoffSteps;
    uint32_t diagnosticIntervalMs;
    int32_t xConfiguredAxisRangeSteps;
    int32_t yConfiguredAxisRangeSteps;
    uint32_t defaultSpeedHz;
    uint32_t defaultAcceleration;
};

using PrintDriverSampleCallback = void (*)();

class HomingStateMachine
{
public:
    HomingStateMachine(
        const HomingStateMachineConfig &config,
        MotorAdapter *&activeMotor,
        MotorAdapter &xMotor,
        MotorAdapter &yMotor,
        StallDetector &stallDetector,
        TestController &testController,
        PrintDriverSampleCallback printDriverSample)
        : config(config),
          activeMotor(activeMotor),
          xMotor(xMotor),
          yMotor(yMotor),
          stallDetector(stallDetector),
          testController(testController),
          printDriverSample(printDriverSample)
    {
    }

    void begin(bool configurationComplete);
    void update();
    void abort(const __FlashStringHelper *reason);
    void fail(const __FlashStringHelper *reason);
    void setFaulted();

    bool isIdle() const
    {
        return state == HomingState::Idle;
    }

    bool isFaulted() const
    {
        return state == HomingState::Fault;
    }

private:
    const HomingStateMachineConfig &config;
    MotorAdapter *&activeMotor;
    MotorAdapter &xMotor;
    MotorAdapter &yMotor;
    StallDetector &stallDetector;
    TestController &testController;
    PrintDriverSampleCallback printDriverSample;
    HomingState state = HomingState::Idle;
    HomingPhase phase = HomingPhase::FindZero;
    uint32_t lastDiagnosticMs = 0;

    void finishCurrentAxis();
    void printMotionSample();
    bool startMove(int32_t steps);
    bool startMoveTo(int32_t position);
    void startSeekingCurrentPhase();
    int32_t seekStepsForCurrentPhase() const;
    int32_t backoffStepsForCurrentPhase() const;
    int32_t configuredAxisRangeSteps(const Axis &axis) const;
};