#include "CommandSystem.h"

#include "HomingStateMachine.h"

#include <stdlib.h>
#include <string.h>

namespace
{
    const CommandDescriptor commandDescriptors[] = {
        {CommandGroup::Homing, CommandId::StartHoming, CommandParameterId::None, 0, 's', "homing.start", F("s = start homing")},
        {CommandGroup::RobotMotion, CommandId::AbortActive, CommandParameterId::None, 0, 'x', "motion.abort", F("x = abort active homing/range/pattern")},
        {CommandGroup::RobotMotion, CommandId::MoveToPosition, CommandParameterId::NormalizedPosition, 0, 'm', "motion.position", F("m x y = move to normalized position 0..10000")},
        {CommandGroup::RobotMotion, CommandId::MoveToZero, CommandParameterId::None, 0, 'z', "motion.zero", F("z = move to normalized position 0 0")},
        {CommandGroup::RobotMotion, CommandId::SetMicrosteps, CommandParameterId::Microsteps, 0, 'u', "motion.microsteps", F("u n = set runtime microsteps 1,2,4,8,16,32,64,128,256")},
        {CommandGroup::RobotMotion, CommandId::SetSpeed, CommandParameterId::Speed, 0, 'v', "motion.speed", F("v hz = set runtime speed 1..50000 steps/s")},
        {CommandGroup::RobotMotion, CommandId::SetAcceleration, CommandParameterId::Acceleration, 0, 'a', "motion.acceleration", F("a n = set runtime acceleration 1..100000 steps/s^2")},
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

    bool parseUnsignedValue(char *cursor, unsigned long maxValue, uint32_t &value)
    {
        cursor = skipSeparators(cursor);
        char *end = cursor;
        const unsigned long parsed = strtoul(cursor, &end, 10);
        if (end == cursor || parsed > maxValue)
        {
            return false;
        }

        cursor = skipSeparators(end);
        if (*cursor != '\0')
        {
            return false;
        }

        value = static_cast<uint32_t>(parsed);
        return true;
    }

    bool isSupportedMicrosteps(uint32_t microsteps)
    {
        return microsteps == 1 || microsteps == 2 || microsteps == 4 || microsteps == 8 ||
               microsteps == 16 || microsteps == 32 || microsteps == 64 || microsteps == 128 ||
               microsteps == 256;
    }

    int32_t scaleRounded(int32_t value, uint16_t fromMicrosteps, uint16_t toMicrosteps)
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
            if (descriptor != nullptr && descriptor->parameter != CommandParameterId::NormalizedPosition &&
                descriptor->parameter != CommandParameterId::Microsteps &&
                descriptor->parameter != CommandParameterId::Speed &&
                descriptor->parameter != CommandParameterId::Acceleration)
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
    const char commandKey = normalizeSerialKey(*cursor);
    if (commandKey != 'm' && commandKey != 'u' && commandKey != 'v' && commandKey != 'a')
    {
        return false;
    }

    cursor++;
    if (commandKey != 'm')
    {
        uint32_t value = 0;
        if (!parseUnsignedValue(cursor, 100000, value))
        {
            return false;
        }

        const CommandDescriptor *descriptor = catalog.findBySerialKey(commandKey);
        if (descriptor == nullptr)
        {
            return false;
        }

        command = commandFromDescriptor(*descriptor);
        command.parameter.value = value;
        return true;
    }

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
    case CommandId::MoveToZero:
        return moveToZero();
    case CommandId::SetMicrosteps:
        return setMicrosteps(command.parameter);
    case CommandId::SetSpeed:
        return setSpeed(command.parameter);
    case CommandId::SetAcceleration:
        return setAcceleration(command.parameter);
    case CommandId::PrintStallGuardSample:
        printDriverSample();
        return {CommandStatus::Accepted, F("StallGuard sample printed.")};
    case CommandId::StartRangeTest:
        if (!homing.isIdle())
        {
            output.println(F("Range test refused: homing is active or faulted."));
            return {CommandStatus::Busy, F("Homing is active or faulted.")};
        }
        if (testController.isActive())
        {
            output.println(F("Range test refused: a test is already active."));
            return {CommandStatus::Busy, F("A test is already active.")};
        }
        testController.beginRangeTest();
        if (!testController.isActive())
        {
            return {CommandStatus::Rejected, F("Range test could not start.")};
        }
        return {CommandStatus::Accepted, F("Range test command processed.")};
    case CommandId::StartPatternTest:
        return startPatternTest(command.parameter);
    case CommandId::PrintCommandCatalog:
        catalog.printTo(output);
        return {CommandStatus::Accepted, F("Command catalog printed.")};
    }

    return {CommandStatus::UnknownCommand, F("Unknown command.")};
}

bool CommandDispatcher::motorsAreIdle() const
{
    return !xMotor.isRunning() && !yMotor.isRunning();
}

bool CommandDispatcher::applyRuntimeMotionProfile()
{
    return xMotor.setProfile(runtimeMotionSettings.speedHz, runtimeMotionSettings.acceleration) &&
           yMotor.setProfile(runtimeMotionSettings.speedHz, runtimeMotionSettings.acceleration);
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

    if (!motorsAreIdle())
    {
        output.println(F("Move refused: motors are active."));
        return {CommandStatus::Busy, F("Motors are active.")};
    }

    if (xMotor.axisState().axisRangeSteps <= 0 || yMotor.axisState().axisRangeSteps <= 0)
    {
        output.println(F("Move refused: run homing first so both axis ranges are known."));
        return {CommandStatus::Rejected, F("Axis ranges are unknown.")};
    }

    const int32_t targetX = positionFromNormalized(xMotor.axisState(), parameter.xPosition);
    const int32_t targetY = positionFromNormalized(yMotor.axisState(), parameter.yPosition);

    if (!applyRuntimeMotionProfile())
    {
        output.println(F("Move refused: invalid runtime speed or acceleration."));
        return {CommandStatus::Rejected, F("Invalid runtime motion profile.")};
    }

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

CommandResult CommandDispatcher::moveToZero()
{
    return moveToPosition({CommandParameterId::NormalizedPosition, 0, 0, 0});
}

CommandResult CommandDispatcher::setMicrosteps(const CommandParameter &parameter)
{
    if (parameter.id != CommandParameterId::Microsteps || !isSupportedMicrosteps(parameter.value))
    {
        output.println(F("Microsteps refused: use 1,2,4,8,16,32,64,128,256."));
        return {CommandStatus::InvalidParameter, F("Invalid microsteps.")};
    }

    if (!homing.isIdle() || testController.isActive() || !motorsAreIdle())
    {
        output.println(F("Microsteps refused: robot is active."));
        return {CommandStatus::Busy, F("Robot is active.")};
    }

    const uint16_t newMicrosteps = static_cast<uint16_t>(parameter.value);
    const uint16_t previousMicrosteps = runtimeMotionSettings.microsteps;
    if (newMicrosteps == previousMicrosteps)
    {
        output.println(F("Runtime microsteps unchanged."));
        return {CommandStatus::Accepted, F("Microsteps unchanged.")};
    }

    xMotor.setMicrosteps(newMicrosteps);
    yMotor.setMicrosteps(newMicrosteps);
    rescaleAxisForMicrosteps(xMotor, previousMicrosteps, newMicrosteps);
    rescaleAxisForMicrosteps(yMotor, previousMicrosteps, newMicrosteps);
    runtimeMotionSettings.microsteps = newMicrosteps;

    output.print(F("Runtime microsteps set to "));
    output.println(runtimeMotionSettings.microsteps);
    return {CommandStatus::Accepted, F("Microsteps command processed.")};
}

CommandResult CommandDispatcher::setSpeed(const CommandParameter &parameter)
{
    if (parameter.id != CommandParameterId::Speed || parameter.value < MIN_RUNTIME_SPEED_HZ || parameter.value > MAX_RUNTIME_SPEED_HZ)
    {
        output.println(F("Speed refused: value must be 1..50000."));
        return {CommandStatus::InvalidParameter, F("Invalid speed.")};
    }

    if (!homing.isIdle() || testController.isActive() || !motorsAreIdle())
    {
        output.println(F("Speed refused: robot is active."));
        return {CommandStatus::Busy, F("Robot is active.")};
    }

    const uint32_t previousSpeedHz = runtimeMotionSettings.speedHz;
    runtimeMotionSettings.speedHz = parameter.value;
    if (!applyRuntimeMotionProfile())
    {
        runtimeMotionSettings.speedHz = previousSpeedHz;
        applyRuntimeMotionProfile();
        output.println(F("Speed refused: FastAccelStepper rejected the profile."));
        return {CommandStatus::Rejected, F("Speed rejected.")};
    }

    output.print(F("Runtime speed set to "));
    output.println(runtimeMotionSettings.speedHz);
    return {CommandStatus::Accepted, F("Speed command processed.")};
}

CommandResult CommandDispatcher::setAcceleration(const CommandParameter &parameter)
{
    if (parameter.id != CommandParameterId::Acceleration || parameter.value < MIN_RUNTIME_ACCELERATION || parameter.value > MAX_RUNTIME_ACCELERATION)
    {
        output.println(F("Acceleration refused: value must be 1..100000."));
        return {CommandStatus::InvalidParameter, F("Invalid acceleration.")};
    }

    if (!homing.isIdle() || testController.isActive() || !motorsAreIdle())
    {
        output.println(F("Acceleration refused: robot is active."));
        return {CommandStatus::Busy, F("Robot is active.")};
    }

    const uint32_t previousAcceleration = runtimeMotionSettings.acceleration;
    runtimeMotionSettings.acceleration = parameter.value;
    if (!applyRuntimeMotionProfile())
    {
        runtimeMotionSettings.acceleration = previousAcceleration;
        applyRuntimeMotionProfile();
        output.println(F("Acceleration refused: FastAccelStepper rejected the profile."));
        return {CommandStatus::Rejected, F("Acceleration rejected.")};
    }

    output.print(F("Runtime acceleration set to "));
    output.println(runtimeMotionSettings.acceleration);
    return {CommandStatus::Accepted, F("Acceleration command processed.")};
}

void CommandDispatcher::rescaleAxisForMicrosteps(MotorAdapter &motor, uint16_t fromMicrosteps, uint16_t toMicrosteps)
{
    Axis &axis = motor.axisState();
    axis.physicalAxisRangeSteps = scaleRounded(axis.physicalAxisRangeSteps, fromMicrosteps, toMicrosteps);
    axis.axisRangeSteps = scaleRounded(axis.axisRangeSteps, fromMicrosteps, toMicrosteps);
    motor.setPosition(scaleRounded(motor.position(), fromMicrosteps, toMicrosteps));
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

    if (testController.isActive())
    {
        output.println(F("Pattern refused: a test is already active."));
        return {CommandStatus::Busy, F("A test is already active.")};
    }

    testController.beginPatternTest(toPatternKind(static_cast<CommandPattern>(parameter.value)));
    if (!testController.isActive())
    {
        return {CommandStatus::Rejected, F("Pattern test could not start.")};
    }
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