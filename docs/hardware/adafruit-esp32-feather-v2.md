# Adafruit ESP32 Feather V2 Pin Capabilities

This board is supported by the `adafruit_feather_esp32_v2` PlatformIO environment. Board-specific firmware constants are selected in [src/BoardConfiguration.h](../../src/BoardConfiguration.h) through the `ROBOT_BOARD_ADAFRUIT_ESP32_FEATHER_V2` build flag.

FastAccelStepper is initialized with CPU core `0` for this target. On the classic dual-core ESP32 used by the Feather V2, Arduino `setup()`/`loop()` normally run on core `1`, so the stepper task is pinned to the other core.

Source reference: Adafruit ESP32 Feather V2 pinout, product 5400.

## Firmware Pin Map

| Signal               | Feather pin |   GPIO | Notes                                                                       |
| -------------------- | ----------- | -----: | --------------------------------------------------------------------------- |
| TMC2209 UART RX      | RX          |  GPIO7 | Firmware receive pin for the shared TMC UART bus.                           |
| TMC2209 UART TX      | TX          |  GPIO8 | Firmware transmit pin for the shared TMC UART bus.                          |
| X STEP               | D32         | GPIO32 | Output-capable ADC1 pin; avoids input-only and strapping pins.              |
| X DIR                | D33         | GPIO33 | Output-capable ADC1 pin; avoids input-only and strapping pins.              |
| Y STEP               | D27         | GPIO27 | Output-capable pin; ADC2 caveat only matters for analog reads during Wi-Fi. |
| Y DIR                | D14         | GPIO14 | Output-capable pin; common SPI-alt capable pin.                             |
| Shared driver enable | A5          |  GPIO4 | Output-capable multifunction pin.                                           |

The mapping intentionally avoids GPIO34, GPIO36, GPIO37, and GPIO39 because they are input-only. It also keeps GPIO12 and GPIO15 unused for motor-control outputs because their strapping behavior can matter during boot.

## Capability Legend

- `yes`: supported or practical for this board pin.
- `caveat`: supported with board or ESP32 caveats.
- `no`: not typically used for this purpose.

Columns:

- `ADC`: analog input.
- `DAC`: analog output.
- `Touch`: capacitive touch channel.
- `PWM`: LEDC PWM output.
- `UART`: can be mapped or used as UART TX/RX/flow through the ESP32 GPIO matrix.
- `I2C`: common or default I2C usage on this board.
- `SPI`: common or default SPI usage on this board.
- `Input Only`: cannot be used as digital output.

## Pin Capability Matrix

| Feather pin |   GPIO | ADC           | DAC       | Touch      | PWM | UART   | I2C      | SPI       | Input only | Notes                                         |
| ----------- | -----: | ------------- | --------- | ---------- | --- | ------ | -------- | --------- | ---------- | --------------------------------------------- |
| A0          | GPIO26 | yes, ADC2_CH9 | yes, DAC2 | no         | yes | yes    | caveat   | caveat    | no         | Also DAC2.                                    |
| A1          | GPIO25 | yes, ADC2_CH8 | yes, DAC1 | no         | yes | yes    | caveat   | caveat    | no         | Also DAC1.                                    |
| A2          | GPIO34 | yes, ADC1_CH6 | no        | no         | no  | caveat | no       | no        | yes        | Input-only pin.                               |
| A3          | GPIO39 | yes, ADC1_CH3 | no        | yes, RTC3  | no  | caveat | no       | no        | yes        | Input-only pin.                               |
| A4          | GPIO36 | yes, ADC1_CH0 | no        | yes, RTC0  | no  | caveat | no       | no        | yes        | Input-only pin.                               |
| A5          |  GPIO4 | yes, ADC2_CH0 | no        | yes, RTC10 | yes | yes    | caveat   | caveat    | no         | Useful multifunction pin.                     |
| SCK         |  GPIO5 | no            | no        | no         | yes | yes    | caveat   | yes, SCK  | no         | Default SPI clock.                            |
| MO          | GPIO19 | no            | no        | no         | yes | yes    | caveat   | yes, MOSI | no         | Default SPI MOSI.                             |
| MI          | GPIO21 | no            | no        | no         | yes | yes    | caveat   | yes, MISO | no         | Default SPI MISO.                             |
| RX          |  GPIO7 | no            | no        | no         | yes | yes    | caveat   | caveat    | no         | Labeled RX on Feather header.                 |
| TX          |  GPIO8 | no            | no        | no         | yes | yes    | caveat   | caveat    | no         | Labeled TX on Feather header.                 |
| D37         | GPIO37 | yes, ADC1_CH1 | no        | yes, RTC1  | no  | caveat | no       | no        | yes        | Input-only pin.                               |
| D12         | GPIO12 | yes, ADC2_CH5 | no        | yes, RTC15 | yes | yes    | caveat   | yes       | no         | Strapping-related on ESP32; use care at boot. |
| D27         | GPIO27 | yes, ADC2_CH7 | no        | yes, RTC17 | yes | yes    | caveat   | caveat    | no         | General GPIO/ADC2 pin.                        |
| D33         | GPIO33 | yes, ADC1_CH5 | no        | yes, RTC8  | yes | yes    | caveat   | caveat    | no         | General-purpose pin.                          |
| D15         | GPIO15 | yes, ADC2_CH3 | no        | yes, RTC13 | yes | yes    | caveat   | yes       | no         | Strapping caveats apply.                      |
| D32         | GPIO32 | yes, ADC1_CH4 | no        | yes, RTC9  | yes | yes    | caveat   | caveat    | no         | Flexible analog and digital pin.              |
| D14         | GPIO14 | yes, ADC2_CH6 | no        | yes, RTC16 | yes | yes    | caveat   | yes       | no         | Common SPI-alt capable pin.                   |
| SCL         | GPIO20 | no            | no        | no         | yes | yes    | yes, SCL | caveat    | no         | Default I2C SCL.                              |
| SDA         | GPIO22 | no            | no        | no         | yes | yes    | yes, SDA | caveat    | no         | Default I2C SDA.                              |

## Quick Pin Notes

### D15

- ADC: yes, ADC2_CH3.
- Touch: yes.
- PWM: yes.
- UART: yes.
- SPI alternate use: yes.
- Caveat: ESP32 strapping behavior can matter during boot depending on external circuitry.

### D32

- ADC: yes, ADC1_CH4.
- Touch: yes.
- PWM: yes.
- UART: yes.
- Good flexible GPIO for future board mappings.

### SCL and SDA

- Best kept as the default I2C pins unless the future board design intentionally repurposes them.
- SCL is GPIO20.
- SDA is GPIO22.

## Important Caveats

1. ADC2 and Wi-Fi: classic ESP32 ADC2 channels can be unavailable or contended while Wi-Fi is active. Prefer ADC1 pins for analog reads when Wi-Fi is enabled.
2. Input-only pins: GPIO34, GPIO36, GPIO37, and GPIO39 cannot be used as digital output or PWM.
3. Strapping and special pins: GPIO12, GPIO15, and JTAG-related signals can affect boot or debug behavior if externally driven at reset.
4. Peripheral flexibility: ESP32 GPIO matrix remapping is flexible, but board defaults are usually easier to maintain.

## Porting Notes

1. Keep motor step, direction, enable, PWM, and any other output signal off the input-only pins.
2. Treat ADC2 pins carefully if the future Web API uses Wi-Fi.
3. Revisit the pin map if external circuitry drives GPIO12 or GPIO15 at reset.
