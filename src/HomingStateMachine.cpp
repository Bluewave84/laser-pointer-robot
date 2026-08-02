#include "HomingStateMachine.h"

void HomingStateMachine::printMotionSample()
{
    if (millis() - lastDiagnosticMs < config.diagnosticIntervalMs)
    {
        return;
    }

    lastDiagnosticMs = millis();
    Serial.print(F("[HOME] position="));
    Serial.print(activeMotor->position());
    Serial.print(F(" axis="));
    Serial.print(activeMotor->axisName());
    Serial.print(F(" state="));
    Serial.print(static_cast<uint8_t>(state));
    Serial.print(' ');
    printDriverSample();
}

void HomingStateMachine::fail(const __FlashStringHelper *reason)
{
    if (activeMotor->isRunning())
    {
        activeMotor->forceStop();
    }

    state = HomingState::Fault;
    Serial.print(F("Homing fault: "));
    Serial.println(reason);
    printDriverSample();
}

void HomingStateMachine::setFaulted()
{
    state = HomingState::Fault;
}

bool HomingStateMachine::startMove(int32_t steps)
{
    if (!activeMotor->move(steps))
    {
        fail(F("FastAccelStepper rejected the move"));
        return false;
    }

    return true;
}

bool HomingStateMachine::startMoveTo(int32_t position)
{
    if (!activeMotor->moveTo(position))
    {
        Serial.println(F("Range test fault: FastAccelStepper rejected the moveTo command."));
        return false;
    }

    return true;
}

static bool axisRangeIsKnown(const Axis &axis)
{
    return axis.axisRangeSteps > 0;
}

static int32_t scaleRounded(int32_t value, uint16_t fromMicrosteps, uint16_t toMicrosteps)
{
    if (value == 0 || fromMicrosteps == toMicrosteps)
    {
        return value;
    }

    const int64_t numerator = static_cast<int64_t>(value) * toMicrosteps;
    const int64_t adjustment = fromMicrosteps / 2;
    if (numerator >= 0)
    {
        return static_cast<int32_t>((numerator + adjustment) / fromMicrosteps);
    }

    return static_cast<int32_t>((numerator - adjustment) / fromMicrosteps);
}

int32_t HomingStateMachine::configuredAxisRangeSteps(const Axis &axis) const
{
    if (&axis == &xMotor.axisState())
    {
        return config.xConfiguredAxisRangeSteps;
    }

    return config.yConfiguredAxisRangeSteps;
}

void HomingStateMachine::finishCurrentAxis()
{
    activeMotor->disableOutputs();
    state = HomingState::Idle;
    Serial.print(F("Homing complete. Range 0.."));
    Serial.print(activeMotor->axisState().axisRangeSteps);
    Serial.print(F(" steps, final position="));
    Serial.println(activeMotor->position());

    if (activeMotor == &xMotor)
    {
        activeMotor = &yMotor;
        lastDiagnosticMs = 0;
        phase = HomingPhase::FindZero;
        state = HomingState::Arming;
        Serial.println(F("Homing: starting Y axis."));
        Serial.println(F("Homing: arming before seeking zero end."));
        startMove(config.armingSteps);
    }
    else
    {
        restoreRuntimeMotionSettings();
    }
}

void HomingStateMachine::restoreRuntimeMotionSettings()
{
    xMotor.setMicrosteps(runtimeMotionSettings.microsteps);
    yMotor.setMicrosteps(runtimeMotionSettings.microsteps);
    rescaleAxisForRuntimeMicrosteps(xMotor);
    rescaleAxisForRuntimeMicrosteps(yMotor);
    xMotor.setProfile(runtimeMotionSettings.speedHz, runtimeMotionSettings.acceleration);
    yMotor.setProfile(runtimeMotionSettings.speedHz, runtimeMotionSettings.acceleration);
    Serial.print(F("Homing: restored runtime motion settings microsteps="));
    Serial.print(runtimeMotionSettings.microsteps);
    Serial.print(F(" speed="));
    Serial.print(runtimeMotionSettings.speedHz);
    Serial.print(F(" acceleration="));
    Serial.println(runtimeMotionSettings.acceleration);
}

void HomingStateMachine::rescaleAxisForRuntimeMicrosteps(MotorAdapter &motor)
{
    Axis &axis = motor.axisState();
    axis.physicalAxisRangeSteps = scaleRounded(axis.physicalAxisRangeSteps, config.homingMicrosteps, runtimeMotionSettings.microsteps);
    axis.axisRangeSteps = scaleRounded(axis.axisRangeSteps, config.homingMicrosteps, runtimeMotionSettings.microsteps);
    motor.setPosition(scaleRounded(motor.position(), config.homingMicrosteps, runtimeMotionSettings.microsteps));
}

int32_t HomingStateMachine::seekStepsForCurrentPhase() const
{
    return phase == HomingPhase::FindZero ? -config.maxSeekSteps : config.maxSeekSteps;
}

int32_t HomingStateMachine::backoffStepsForCurrentPhase() const
{
    return phase == HomingPhase::FindZero ? config.backoffSteps : -config.backoffSteps;
}

void HomingStateMachine::startSeekingCurrentPhase()
{
    stallDetector.beginSeek(activeMotor->position());
    state = HomingState::Seeking;

    if (phase == HomingPhase::FindZero)
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

void HomingStateMachine::begin(bool configurationComplete)
{
    if (testController.isActive())
    {
        Serial.println(F("Homing refused: a test is active."));
        return;
    }

    if (state != HomingState::Idle && state != HomingState::Fault)
    {
        Serial.println(F("Homing is already active."));
        return;
    }

    if (!configurationComplete)
    {
        Serial.println(F("Set R_SENSE_OHMS, MOTOR_RMS_CURRENT_MA, STALLGUARD_TCOOLTHRS, and STALLGUARD_SGTHRS before homing."));
        return;
    }

    lastDiagnosticMs = 0;
    xMotor.setMicrosteps(config.homingMicrosteps);
    yMotor.setMicrosteps(config.homingMicrosteps);
    xMotor.setProfile(config.defaultSpeedHz, config.defaultAcceleration);
    yMotor.setProfile(config.defaultSpeedHz, config.defaultAcceleration);
    xMotor.axisState().physicalAxisRangeSteps = 0;
    xMotor.axisState().axisRangeSteps = 0;
    yMotor.axisState().physicalAxisRangeSteps = 0;
    yMotor.axisState().axisRangeSteps = 0;

    xMotor.axisState().axisRangeSteps = configuredAxisRangeSteps(xMotor.axisState());
    if (xMotor.axisState().axisRangeSteps > 0)
    {
        Serial.print(F("Homing: X configured range set to 0.."));
        Serial.print(xMotor.axisState().axisRangeSteps);
        Serial.println(F(" steps (FindMax will be skipped)."));
    }
    else
    {
        Serial.println(F("Homing: X range unknown; full FindZero + FindMax homing will run."));
    }

    yMotor.axisState().axisRangeSteps = configuredAxisRangeSteps(yMotor.axisState());
    if (yMotor.axisState().axisRangeSteps > 0)
    {
        Serial.print(F("Homing: Y configured range set to 0.."));
        Serial.print(yMotor.axisState().axisRangeSteps);
        Serial.println(F(" steps (FindMax will be skipped)."));
    }
    else
    {
        Serial.println(F("Homing: Y range unknown; full FindZero + FindMax homing will run."));
    }

    activeMotor = &xMotor;
    phase = HomingPhase::FindZero;
    state = HomingState::Arming;
    Serial.println(F("Homing: starting X axis."));
    Serial.println(F("Homing: arming before seeking zero end."));
    startMove(config.armingSteps);
}

void HomingStateMachine::abort(const __FlashStringHelper *reason)
{
    fail(reason);
}

void HomingStateMachine::update()
{
    if (state == HomingState::Idle)
    {
        return;
    }

    if (state == HomingState::Fault)
    {
        if (!activeMotor->isRunning())
        {
            activeMotor->disableOutputs();
        }
        return;
    }

    if (state == HomingState::Arming)
    {
        if (activeMotor->isRunning())
        {
            return;
        }

        startSeekingCurrentPhase();
        return;
    }

    if (state == HomingState::Seeking)
    {
        if (!activeMotor->isRunning())
        {
            fail(F("maximum seek travel reached without a confirmed StallGuard hit"));
            return;
        }

        if (stallDetector.confirmed(*activeMotor))
        {
            Serial.println(F("Homing: StallGuard SG_RESULT threshold accepted."));
            Serial.print(F("[HOME] accepted_position="));
            Serial.println(activeMotor->position());
            printDriverSample();
            activeMotor->forceStop();
            state = HomingState::StoppingAtStall;
        }

        printMotionSample();
        return;
    }

    if (state == HomingState::StoppingAtStall)
    {
        if (activeMotor->isRunning())
        {
            return;
        }

        if (phase == HomingPhase::FindZero)
        {
            activeMotor->setPosition(0);
            Serial.println(F("Homing: zero end found at position 0."));
        }
        else
        {
            activeMotor->axisState().physicalAxisRangeSteps = activeMotor->position();
            Serial.print(F("Homing: max end found. Physical range steps="));
            Serial.println(activeMotor->axisState().physicalAxisRangeSteps);
        }

        state = HomingState::BackingOff;
        Serial.println(F("Homing: backing off from the detected end stop."));
        startMove(backoffStepsForCurrentPhase());
        return;
    }

    if (state == HomingState::BackingOff && !activeMotor->isRunning())
    {
        if (phase == HomingPhase::FindZero)
        {
            if (axisRangeIsKnown(activeMotor->axisState()))
            {
                activeMotor->setPosition(0);
                Serial.print(F("Homing: using configured upper limit for axis "));
                Serial.print(activeMotor->axisName());
                Serial.print(F(". Max position set to "));
                Serial.print(activeMotor->axisState().axisRangeSteps);
                Serial.println(F(" steps; FindMax skipped."));
                finishCurrentAxis();
                return;
            }

            phase = HomingPhase::FindMax;
            lastDiagnosticMs = 0;
            Serial.print(F("Homing: zero backoff position="));
            Serial.println(activeMotor->position());
            startSeekingCurrentPhase();
            return;
        }

        if (activeMotor->axisState().physicalAxisRangeSteps <= (config.backoffSteps * 2))
        {
            fail(F("measured range is smaller than both backoff margins"));
            return;
        }

        state = HomingState::MovingToMinimum;
        Serial.print(F("Homing: moving to minimum usable position="));
        Serial.println(config.backoffSteps);
        startMoveTo(config.backoffSteps);
        return;
    }

    if (state == HomingState::MovingToMinimum && !activeMotor->isRunning())
    {
        if (!axisRangeIsKnown(activeMotor->axisState()))
        {
            activeMotor->axisState().axisRangeSteps = activeMotor->axisState().physicalAxisRangeSteps - (config.backoffSteps * 2);
        }
        activeMotor->setPosition(0);

        finishCurrentAxis();
    }
}