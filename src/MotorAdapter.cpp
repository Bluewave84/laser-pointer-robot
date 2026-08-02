#include "MotorAdapter.h"

bool MotorAdapter::configureDriver(uint16_t rmsCurrentMa, uint16_t microsteps, uint32_t stallguardTcoolthrs, uint8_t stallguardSgthrs, bool configurationComplete)
{
    driver.begin();

    if (driver.test_connection() != 0)
    {
        Serial.print(axis.name);
        Serial.println(F(" TMC2209 UART connection failed."));
        return false;
    }

    driver.pdn_disable(true);
    driver.mstep_reg_select(true);
    driver.toff(4);
    driver.blank_time(24);

    if (!configurationComplete)
    {
        Serial.print(axis.name);
        Serial.println(F(" driver connected. Configuration is incomplete; motion remains disabled."));
        return true;
    }

    driver.rms_current(rmsCurrentMa);
    driver.microsteps(microsteps);
    driver.en_spreadCycle(false);
    driver.TCOOLTHRS(stallguardTcoolthrs);
    driver.SGTHRS(stallguardSgthrs);
    return true;
}

bool MotorAdapter::setupMotion(FastAccelStepperEngine &engine, uint8_t enablePin, uint32_t defaultSpeedHz, uint32_t defaultAcceleration)
{
    stepper = engine.stepperConnectToPin(stepPin);
    if (stepper == nullptr)
    {
        Serial.print(axis.name);
        Serial.println(F(" FastAccelStepper initialization failed."));
        return false;
    }

    stepper->setDirectionPin(dirPin, true, 10);
    stepper->setEnablePin(enablePin, true);
    stepper->setAutoEnable(true);
    stepper->setDelayToEnable(2000);
    stepper->setDelayToDisable(100);

    if (!setProfile(defaultSpeedHz, defaultAcceleration))
    {
        Serial.print(axis.name);
        Serial.println(F(" invalid FastAccelStepper speed or acceleration."));
        return false;
    }

    return true;
}

bool MotorAdapter::isInitialized() const
{
    return stepper != nullptr;
}

bool MotorAdapter::isRunning() const
{
    return stepper != nullptr && stepper->isRunning();
}

bool MotorAdapter::move(int32_t steps)
{
    return stepper != nullptr && stepper->move(steps) == MOVE_OK;
}

bool MotorAdapter::moveTo(int32_t position)
{
    return stepper != nullptr && stepper->moveTo(position) == MOVE_OK;
}

void MotorAdapter::forceStop()
{
    if (stepper != nullptr)
    {
        stepper->forceStop();
    }
}

void MotorAdapter::disableOutputs()
{
    if (stepper != nullptr)
    {
        stepper->disableOutputs();
    }
}

int32_t MotorAdapter::position() const
{
    return stepper == nullptr ? 0 : stepper->getCurrentPosition();
}

void MotorAdapter::setPosition(int32_t position)
{
    if (stepper != nullptr)
    {
        stepper->setCurrentPosition(position);
    }
}

bool MotorAdapter::setProfile(uint32_t speedHz, uint32_t acceleration)
{
    return stepper != nullptr &&
           stepper->setSpeedInHz(speedHz) == 0 &&
           stepper->setAcceleration(acceleration) == 0;
}

void MotorAdapter::setMicrosteps(uint16_t microsteps)
{
    driver.microsteps(microsteps);
}

uint16_t MotorAdapter::sgResult()
{
    return driver.SG_RESULT();
}

uint8_t MotorAdapter::sgThreshold()
{
    return driver.SGTHRS();
}