#pragma once

enum CommandType
{
    COMMAND_NONE = 0,
    COMMAND_SET_AXIS_TARGET,
    COMMAND_SET_MOTION_LIMITS,
    COMMAND_ZERO_CURRENT_POSITION,
    COMMAND_EMERGENCY_STOP,
    COMMAND_STOP_ACTIVE_MOVEMENT
};

struct RobotCommand
{
    CommandType type;
    float axis1_degrees;
    float axis2_degrees;
    int speed_percent;
    int acceleration_percent;
};