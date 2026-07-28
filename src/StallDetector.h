#pragma once

#include <Arduino.h>

#include "MotorAdapter.h"

class StallDetector
{
public:
    StallDetector(uint32_t sampleIntervalMs, uint8_t confirmSampleCount, int32_t seekSettleSteps)
        : sampleIntervalMs(sampleIntervalMs), confirmSampleCount(confirmSampleCount), seekSettleSteps(seekSettleSteps)
    {
    }

    void beginSeek(int32_t startPosition);
    bool confirmed(MotorAdapter &motor);

private:
    const uint32_t sampleIntervalMs;
    const uint8_t confirmSampleCount;
    const int32_t seekSettleSteps;
    uint32_t lastSampleMs = 0;
    uint8_t confirmSamples = 0;
    int32_t seekStartPosition = 0;

    void resetConfirmation();
};