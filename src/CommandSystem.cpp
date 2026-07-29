#include "CommandSystem.h"

#include <string.h>

namespace
{
    const CommandDescriptor commandDescriptors[] = {
        {CommandGroup::Homing, CommandId::StartHoming, CommandParameterId::None, 0, 's', "homing.start", F("s = start homing")},
        {CommandGroup::RobotMotion, CommandId::AbortActive, CommandParameterId::None, 0, 'x', "motion.abort", F("x = abort active homing/range/pattern")},
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
            {descriptor.parameter, descriptor.parameterValue},
        };
    }
}

const CommandDescriptor *CommandCatalog::findBySerialKey(char key) const
{
    const char normalizedKey = normalizeSerialKey(key);
    for (uint8_t index = 0; index < commandDescriptorCount; index++)
    {
        if (commandDescriptors[index].serialKey == normalizedKey)
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
    if (serial.available() <= 0)
    {
        return false;
    }

    const char key = static_cast<char>(serial.read());
    const CommandDescriptor *descriptor = catalog.findBySerialKey(key);
    if (descriptor == nullptr)
    {
        return false;
    }

    command = commandFromDescriptor(*descriptor);
    return true;
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