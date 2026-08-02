#pragma once

#include <Arduino.h>

constexpr uint16_t MIN_RUNTIME_MICROSTEPS = 1;
constexpr uint16_t MAX_RUNTIME_MICROSTEPS = 256;
constexpr uint32_t MIN_RUNTIME_SPEED_HZ = 1;
constexpr uint32_t MAX_RUNTIME_SPEED_HZ = 50000;
constexpr uint32_t MIN_RUNTIME_ACCELERATION = 1;
constexpr uint32_t MAX_RUNTIME_ACCELERATION = 100000;

struct RuntimeMotionSettings
{
    uint16_t microsteps;
    uint32_t speedHz;
    uint32_t acceleration;
};