#pragma once

#include <Arduino.h>

struct BoardConfiguration
{
    const char *name;
    int tmcRxPin;
    int tmcTxPin;
    uint8_t xStepPin;
    uint8_t xDirPin;
    uint8_t yStepPin;
    uint8_t yDirPin;
    uint8_t enablePin;
    uint8_t fastAccelStepperCore;
};

#if defined(ROBOT_BOARD_UM_FEATHERS2)
constexpr BoardConfiguration BOARD_CONFIGURATION = {
    "Unexpected Maker FeatherS2",
    44,
    43,
    17,
    18,
    5,
    6,
    12,
    255,
};
#elif defined(ROBOT_BOARD_ADAFRUIT_ESP32_FEATHER_V2)
constexpr BoardConfiguration BOARD_CONFIGURATION = {
    "Adafruit ESP32 Feather V2",
    7,
    8,
    32, // X_STEP_PIN
    33, // X_DIR_PIN
    27, // Y_STEP_PIN
    14, // Y_DIR_PIN
    4,  // ENABLE_PIN
    0,
};
#else
#error "Select a supported robot board in platformio.ini build_flags."
#endif