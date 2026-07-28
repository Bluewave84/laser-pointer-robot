#include "StallDetector.h"

void StallDetector::beginSeek(int32_t startPosition)
{
    seekStartPosition = startPosition;
    resetConfirmation();
}

void StallDetector::resetConfirmation()
{
    lastSampleMs = 0;
    confirmSamples = 0;
}

bool StallDetector::confirmed(MotorAdapter &motor)
{
    if (abs(motor.position() - seekStartPosition) < seekSettleSteps)
    {
        resetConfirmation();
        return false;
    }

    if (millis() - lastSampleMs < sampleIntervalMs)
    {
        return false;
    }

    lastSampleMs = millis();
    const uint16_t sgResult = motor.sgResult();
    if (sgResult <= motor.sgThreshold())
    {
        confirmSamples++;
    }
    else
    {
        confirmSamples = 0;
    }

    return confirmSamples >= confirmSampleCount;
}