#pragma once

#include <Arduino.h>

#include "MotorAdapter.h"
#include "TestController.h"

class HomingStateMachine;

constexpr uint16_t COMMAND_POSITION_SCALE = 10000;
constexpr uint8_t SERIAL_COMMAND_BUFFER_LENGTH = 32;

enum class CommandGroup : uint8_t
{
    Homing,
    RobotMotion,
    TestController,
    StallDetection,
    Catalog,
};

enum class CommandId : uint8_t
{
    StartHoming,
    AbortActive,
    MoveToPosition,
    PrintStallGuardSample,
    StartRangeTest,
    StartPatternTest,
    PrintCommandCatalog,
};

enum class CommandParameterId : uint8_t
{
    None,
    Pattern,
    NormalizedPosition,
};

enum class CommandPattern : uint8_t
{
    Square,
    Diamond,
    Figure8,
    Spiral,
};

struct CommandParameter
{
    CommandParameterId id;
    uint8_t value;
    uint16_t xPosition;
    uint16_t yPosition;
};

struct Command
{
    CommandGroup group;
    CommandId id;
    CommandParameter parameter;
};

Command moveToPositionCommand(uint16_t xPosition, uint16_t yPosition);

enum class CommandStatus : uint8_t
{
    Accepted,
    Rejected,
    UnknownCommand,
    InvalidParameter,
    Busy,
    Faulted,
};

struct CommandResult
{
    CommandStatus status;
    const __FlashStringHelper *message;
};

struct CommandDescriptor
{
    CommandGroup group;
    CommandId id;
    CommandParameterId parameter;
    uint8_t parameterValue;
    char serialKey;
    const char *webName;
    const __FlashStringHelper *help;
};

class CommandCatalog
{
public:
    const CommandDescriptor *findBySerialKey(char key) const;
    const CommandDescriptor *findByWebName(const char *webName) const;
    const CommandDescriptor *findById(CommandGroup group, CommandId id) const;
    const CommandDescriptor *commands(uint8_t &count) const;
    void printTo(Print &output) const;
};

class SerialCommandInput
{
public:
    SerialCommandInput(Stream &serial, const CommandCatalog &catalog)
        : serial(serial), catalog(catalog)
    {
    }

    bool read(Command &command);

private:
    Stream &serial;
    const CommandCatalog &catalog;
    char lineBuffer[SERIAL_COMMAND_BUFFER_LENGTH] = {};
    uint8_t lineLength = 0;

    bool parseLineCommand(Command &command);
    void resetLine();
};

using ConfigurationIsCompleteCallback = bool (*)();
using PrintDriverSampleCommandCallback = void (*)();

class CommandDispatcher
{
public:
    CommandDispatcher(
        HomingStateMachine &homing,
        TestController &testController,
        MotorAdapter &xMotor,
        MotorAdapter &yMotor,
        const CommandCatalog &catalog,
        Print &output,
        ConfigurationIsCompleteCallback configurationIsComplete,
        PrintDriverSampleCommandCallback printDriverSample)
        : homing(homing),
          testController(testController),
          xMotor(xMotor),
          yMotor(yMotor),
          catalog(catalog),
          output(output),
          configurationIsComplete(configurationIsComplete),
          printDriverSample(printDriverSample)
    {
    }

    CommandResult dispatch(const Command &command);

private:
    HomingStateMachine &homing;
    TestController &testController;
    MotorAdapter &xMotor;
    MotorAdapter &yMotor;
    const CommandCatalog &catalog;
    Print &output;
    ConfigurationIsCompleteCallback configurationIsComplete;
    PrintDriverSampleCommandCallback printDriverSample;

    CommandResult moveToPosition(const CommandParameter &parameter);
    int32_t positionFromNormalized(const Axis &axis, uint16_t normalizedPosition) const;
    CommandResult startPatternTest(const CommandParameter &parameter);
    PatternKind toPatternKind(CommandPattern pattern) const;
};