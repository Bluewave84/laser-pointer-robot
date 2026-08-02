#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>

struct Axis
{
    const char *name;
    int32_t physicalAxisRangeSteps;
    int32_t axisRangeSteps;
};

class MotorAdapter
{
public:
    MotorAdapter(Axis &axis, TMC2209Stepper &driver, FastAccelStepper *&stepper, uint8_t stepPin, uint8_t dirPin)
        : axis(axis), driver(driver), stepper(stepper), stepPin(stepPin), dirPin(dirPin)
    {
    }

    Axis &axisState()
    {
        return axis;
    }

    const Axis &axisState() const
    {
        return axis;
    }

    const char *axisName() const
    {
        return axis.name;
    }

    bool configureDriver(uint16_t rmsCurrentMa, uint16_t microsteps, uint32_t stallguardTcoolthrs, uint8_t stallguardSgthrs, bool configurationComplete);
    bool setupMotion(FastAccelStepperEngine &engine, uint8_t enablePin, uint32_t defaultSpeedHz, uint32_t defaultAcceleration);
    bool isInitialized() const;
    bool isRunning() const;
    bool move(int32_t steps);
    bool moveTo(int32_t position);
    void forceStop();
    void disableOutputs();
    int32_t position() const;
    void setPosition(int32_t position);
    bool setProfile(uint32_t speedHz, uint32_t acceleration);
    void setMicrosteps(uint16_t microsteps);
    uint16_t sgResult();
    uint8_t sgThreshold();

private:
    Axis &axis;
    TMC2209Stepper &driver;
    FastAccelStepper *&stepper;
    const uint8_t stepPin;
    const uint8_t dirPin;
};