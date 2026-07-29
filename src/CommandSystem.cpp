#include "CommandSystem.h"

#include <stdlib.h>
#include <string.h>

namespace
{
    const CommandDescriptor commandDescriptors[] = {
        {CommandGroup::Homing, CommandId::StartHoming, CommandParameterId::None, 0, 's', "homing.start", F("s = start homing")},
        {CommandGroup::RobotMotion, CommandId::AbortActive, CommandParameterId::None, 0, 'x', "motion.abort", F("x = abort active homing/range/pattern")},
        {CommandGroup::RobotMotion, CommandId::MoveToPosition, CommandParameterId::NormalizedPosition, 0, 'm', "motion.position", F("m x y = move to normalized position 0..10000")},
        {CommandGroup::StallDetection, CommandId::PrintStallGuardSample, CommandParameterId::None, 0, 'd', "stall.sample", F("d = print one StallGuard diagnostic sample")},
        {CommandGroup::TestController, CommandId::StartRangeTest, CommandParameterId::None, 0, 'c', "test.range", F("c = run axis range sweep")},
        {CommandGroup::TestController, CommandId::StartPatternTest, CommandParameterId::Pattern, static_cast<uint8_t>(CommandPattern::Square), '1', "test.pattern.square", F("1 = square pattern")},
        {CommandGroup::TestController, CommandId::StartPatternTest, CommandParameterId::Pattern, static_cast<uint8_t>(CommandPattern::Diamond), '2', "test.pattern.diamond", F("2 = diamond pattern")},
        {CommandGroup::TestController, CommandId::StartPatternTest, CommandParameterId::Pattern, static_cast<uint8_t>(CommandPattern::Figure8), '3', "test.pattern.figure8", F("3 = figure-8 pattern")},
        {CommandGroup::TestController, CommandId::StartPatternTest, CommandParameterId::Pattern, static_cast<uint8_t>(CommandPattern::Spiral), '4', "test.pattern.spiral", F("4 = spiral pattern")},
        {CommandGroup::Catalog, CommandId::PrintCommandCatalog, CommandParameterId::None, 0, 'p', "catalog.print", F("p = print command catalog")},
    };

    constexpr uint8_t commandDescriptorCount = static_cast<uint8_t>(sizeof(commandDescriptors) / sizeof(commandDescriptors[0]));

    char normalizeSerialKey(char key)
    {
        if (key >= 'A' && key <= 'Z')
        {
            return static_cast<char>(key - 'A' + 'a');
        }

        return key;
    }

    Command commandFromDescriptor(const CommandDescriptor &descriptor)
    {
        return {
            descriptor.group,
            descriptor.id,
            {descriptor.parameter, descriptor.parameterValue, 0, 0},
        };
    }

    bool isSeparator(char value)
    {
        return value == ' ' || value == '\t' || value == ',';
    }

    char *skipSeparators(char *value)
    {
        while (*value != '\0' && isSeparator(*value))
        {
            value++;
        }

        return value;
    }
}

Command moveToPositionCommand(uint16_t xPosition, uint16_t yPosition)
{
    return {
        CommandGroup::RobotMotion,
        CommandId::MoveToPosition,
        {CommandParameterId::NormalizedPosition, 0, xPosition, yPosition},
    };
}

const CommandDescriptor *CommandCatalog::findBySerialKey(char key) const
{
    const char normalizedKey = normalizeSerialKey(key);
    for (uint8_t index = 0; index < commandDescriptorCount; index++)
    {
        if (commandDescriptors[index].serialKey != 0 && commandDescriptors[index].serialKey == normalizedKey)
        {
            return &commandDescriptors[index];
        }
    }

    return nullptr;
}

const CommandDescriptor *CommandCatalog::findByWebName(const char *webName) const
{
    if (webName == nullptr)
    {
        return nullptr;
    }

    for (uint8_t index = 0; index < commandDescriptorCount; index++)
    {
        if (strcmp(commandDescriptors[index].webName, webName) == 0)
        {
            return &commandDescriptors[index];
        }
    }

    return nullptr;
}

const CommandDescriptor *CommandCatalog::findById(CommandGroup group, CommandId id) const
{
    for (uint8_t index = 0; index < commandDescriptorCount; index++)
    {
        if (commandDescriptors[index].group == group && commandDescriptors[index].id == id)
        {
            return &commandDescriptors[index];
        }
    }

    return nullptr;
}

const CommandDescriptor *CommandCatalog::commands(uint8_t &count) const
{
    count = commandDescriptorCount;
    return commandDescriptors;
}

void CommandCatalog::printTo(Print &output) const
{
    output.println(F("Command catalog:"));
    for (uint8_t index = 0; index < commandDescriptorCount; index++)
    {
        output.print(F("  "));
        output.print(commandDescriptors[index].help);
        output.print(F(" ["));
        output.print(commandDescriptors[index].webName);
        output.println(F("]"));
    }
}

bool SerialCommandInput::read(Command &command)
{
    while (serial.available() > 0)
    {
        const char key = static_cast<char>(serial.read());

        if (lineLength == 0 && (key == '\r' || key == '\n' || key == ' ' || key == '\t'))
        {
            continue;
        }

        if (lineLength == 0)
        {
            const CommandDescriptor *descriptor = catalog.findBySerialKey(key);
            if (descriptor != nullptr && descriptor->parameter != CommandParameterId::NormalizedPosition)
            {
                command = commandFromDescriptor(*descriptor);
                return true;
            }
        }

        if (key == '\r' || key == '\n')
        {
            lineBuffer[lineLength] = '\0';
            const bool parsed = parseLineCommand(command);
            resetLine();
            return parsed;
        }

        if (lineLength >= SERIAL_COMMAND_BUFFER_LENGTH - 1)
        {
            resetLine();
            return false;
        }

        lineBuffer[lineLength++] = key;
    }

    return false;
}

bool SerialCommandInput::parseLineCommand(Command &command)
{
    char *cursor = lineBuffer;
    if (normalizeSerialKey(*cursor) != 'm')
    {
        return false;
    }

    cursor++;
    cursor = skipSeparators(cursor);
    char *xEnd = cursor;
    const unsigned long xPosition = strtoul(cursor, &xEnd, 10);
    if (xEnd == cursor)
    {
        return false;
    }

    cursor = skipSeparators(xEnd);
    char *yEnd = cursor;
    const unsigned long yPosition = strtoul(cursor, &yEnd, 10);
    if (yEnd == cursor)
    {
        return false;
    }

    cursor = skipSeparators(yEnd);
    if (*cursor != '\0' || xPosition > COMMAND_POSITION_SCALE || yPosition > COMMAND_POSITION_SCALE)
    {
        return false;
    }

    command = moveToPositionCommand(static_cast<uint16_t>(xPosition), static_cast<uint16_t>(yPosition));
    return true;
}

void SerialCommandInput::resetLine()
{
    lineLength = 0;
    lineBuffer[0] = '\0';
}

CommandResult CommandDispatcher::dispatch(const Command &command)
{
    switch (command.id)
    {
    case CommandId::StartHoming:
        homing.begin(configurationIsComplete());
        return {CommandStatus::Accepted, F("Homing command processed.")};
    case CommandId::AbortActive:
        if (testController.isActive())
        {
            testController.cancel(F("command abort"));
        }
        else
        {
            homing.abort(F("command abort"));
        }
        return {CommandStatus::Accepted, F("Abort command processed.")};
    case CommandId::MoveToPosition:
        return moveToPosition(command.parameter);
    case CommandId::PrintStallGuardSample:
        printDriverSample();
        return {CommandStatus::Accepted, F("StallGuard sample printed.")};
    case CommandId::StartRangeTest:
        if (!homing.isIdle())
        {
            output.println(F("Range test refused: homing is active or faulted."));
            return {CommandStatus::Busy, F("Homing is active or faulted.")};
        }
        testController.beginRangeTest();
        return {CommandStatus::Accepted, F("Range test command processed.")};
    case CommandId::StartPatternTest:
        return startPatternTest(command.parameter);
    case CommandId::PrintCommandCatalog:
        catalog.printTo(output);
        return {CommandStatus::Accepted, F("Command catalog printed.")};
    }

    return {CommandStatus::UnknownCommand, F("Unknown command.")};
}

CommandResult CommandDispatcher::moveToPosition(const CommandParameter &parameter)
{
    if (parameter.id != CommandParameterId::NormalizedPosition)
    {
        output.println(F("Move refused: missing normalized position parameter."));
        return {CommandStatus::InvalidParameter, F("Missing normalized position parameter.")};
    }

    if (parameter.xPosition > COMMAND_POSITION_SCALE || parameter.yPosition > COMMAND_POSITION_SCALE)
    {
        output.println(F("Move refused: normalized positions must be 0..10000."));
        return {CommandStatus::InvalidParameter, F("Normalized position out of range.")};
    }

    if (!homing.isIdle())
    {
        output.println(F("Move refused: homing is active or faulted."));
        return {CommandStatus::Busy, F("Homing is active or faulted.")};
    }

    if (testController.isActive())
    {
        output.println(F("Move refused: a test is active."));
        return {CommandStatus::Busy, F("A test is active.")};
    }

    if (xMotor.axisState().axisRangeSteps <= 0 || yMotor.axisState().axisRangeSteps <= 0)
    {
        output.println(F("Move refused: run homing first so both axis ranges are known."));
        return {CommandStatus::Rejected, F("Axis ranges are unknown.")};
    }

    const int32_t targetX = positionFromNormalized(xMotor.axisState(), parameter.xPosition);
    const int32_t targetY = positionFromNormalized(yMotor.axisState(), parameter.yPosition);

    if (!xMotor.moveTo(targetX))
    {
        output.println(F("Move fault: X moveTo rejected."));
        return {CommandStatus::Rejected, F("X move rejected.")};
    }

    if (!yMotor.moveTo(targetY))
    {
        xMotor.forceStop();
        output.println(F("Move fault: Y moveTo rejected."));
        return {CommandStatus::Rejected, F("Y move rejected.")};
    }

    output.print(F("Move start: x="));
    output.print(targetX);
    output.print(F(" y="));
    output.println(targetY);
    return {CommandStatus::Accepted, F("Move command processed.")};
}

int32_t CommandDispatcher::positionFromNormalized(const Axis &axis, uint16_t normalizedPosition) const
{
    return static_cast<int32_t>((static_cast<int64_t>(axis.axisRangeSteps) * normalizedPosition) / COMMAND_POSITION_SCALE);
}

CommandResult CommandDispatcher::startPatternTest(const CommandParameter &parameter)
{
    if (parameter.id != CommandParameterId::Pattern)
    {
        output.println(F("Pattern refused: missing pattern parameter."));
        return {CommandStatus::InvalidParameter, F("Missing pattern parameter.")};
    }

    if (parameter.value > static_cast<uint8_t>(CommandPattern::Spiral))
    {
        output.println(F("Pattern refused: invalid pattern parameter."));
        return {CommandStatus::InvalidParameter, F("Invalid pattern parameter.")};
    }

    if (!homing.isIdle())
    {
        output.println(F("Pattern refused: homing is active or faulted."));
        return {CommandStatus::Busy, F("Homing is active or faulted.")};
    }

    testController.beginPatternTest(toPatternKind(static_cast<CommandPattern>(parameter.value)));
    return {CommandStatus::Accepted, F("Pattern command processed.")};
}

PatternKind CommandDispatcher::toPatternKind(CommandPattern pattern) const
{
    switch (pattern)
    {
    case CommandPattern::Square:
        return PatternKind::Square;
    case CommandPattern::Diamond:
        return PatternKind::Diamond;
    case CommandPattern::Figure8:
        return PatternKind::Figure8;
    case CommandPattern::Spiral:
        return PatternKind::Spiral;
    }

    return PatternKind::Square;
}