#pragma once

#include <Arduino.h>
#include <WebServer.h>

#include "CommandSystem.h"
#include "HomingStateMachine.h"
#include "MotorAdapter.h"
#include "MotionSettings.h"
#include "TestController.h"

class WebCommandInput
{
public:
    WebCommandInput(
        CommandDispatcher &dispatcher,
        const CommandCatalog &catalog,
        HomingStateMachine &homing,
        TestController &testController,
        MotorAdapter &xMotor,
        MotorAdapter &yMotor,
        const RuntimeMotionSettings &runtimeMotionSettings)
        : dispatcher(dispatcher),
          catalog(catalog),
          homing(homing),
          testController(testController),
          xMotor(xMotor),
          yMotor(yMotor),
          runtimeMotionSettings(runtimeMotionSettings)
    {
    }

    void begin();
    void handleClient();

private:
    WebServer server{80};
    CommandDispatcher &dispatcher;
    const CommandCatalog &catalog;
    HomingStateMachine &homing;
    TestController &testController;
    MotorAdapter &xMotor;
    MotorAdapter &yMotor;
    const RuntimeMotionSettings &runtimeMotionSettings;

    void handleCommands();
    void handleStatus();
    void handleNotFound();
    void handleCommandPost(const String &webName);
    void registerCommandHandlers();
    void sendCorsPreflight();
    void sendJson(uint16_t statusCode, const String &response);
    bool commandFromRequest(const CommandDescriptor &descriptor, Command &command, String &errorMessage);
    void sendCommandResult(const CommandResult &result);
    void sendError(uint16_t statusCode, const __FlashStringHelper *message);
};