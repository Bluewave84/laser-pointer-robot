#include "WebCommandInput.h"

#include <ArduinoJson.h>
#include <WiFi.h>

namespace
{
    const char *parameterName(CommandParameterId parameter)
    {
        switch (parameter)
        {
        case CommandParameterId::None:
            return "none";
        case CommandParameterId::Pattern:
            return "pattern";
        case CommandParameterId::NormalizedPosition:
            return "normalizedPosition";
        case CommandParameterId::Microsteps:
            return "microsteps";
        case CommandParameterId::Speed:
            return "speed";
        case CommandParameterId::Acceleration:
            return "acceleration";
        }

        return "unknown";
    }

    const char *statusName(CommandStatus status)
    {
        switch (status)
        {
        case CommandStatus::Accepted:
            return "accepted";
        case CommandStatus::Rejected:
            return "rejected";
        case CommandStatus::UnknownCommand:
            return "unknownCommand";
        case CommandStatus::InvalidParameter:
            return "invalidParameter";
        case CommandStatus::Busy:
            return "busy";
        case CommandStatus::Faulted:
            return "faulted";
        }

        return "unknown";
    }

    uint16_t httpStatusCode(CommandStatus status)
    {
        switch (status)
        {
        case CommandStatus::Accepted:
            return 202;
        case CommandStatus::UnknownCommand:
        case CommandStatus::InvalidParameter:
            return 400;
        case CommandStatus::Busy:
            return 409;
        case CommandStatus::Rejected:
        case CommandStatus::Faulted:
            return 422;
        }

        return 500;
    }

    String flashStringToString(const __FlashStringHelper *value)
    {
        if (value == nullptr)
        {
            return "";
        }

        return String(value);
    }

    bool isObjectBodyRequired(CommandParameterId parameter)
    {
        return parameter == CommandParameterId::NormalizedPosition ||
               parameter == CommandParameterId::Microsteps ||
               parameter == CommandParameterId::Speed ||
               parameter == CommandParameterId::Acceleration;
    }
}

void WebCommandInput::begin()
{
    server.on("/api/commands", HTTP_GET, [this]()
              { handleCommands(); });
    server.on("/api/commands", HTTP_OPTIONS, [this]()
              { sendCorsPreflight(); });
    server.on("/api/status", HTTP_GET, [this]()
              { handleStatus(); });
    server.on("/api/status", HTTP_OPTIONS, [this]()
              { sendCorsPreflight(); });
    server.on("/", HTTP_GET, [this]()
              { sendError(404, F("No browser UI is served by this firmware. Use /api/status or /api/commands.")); });
    server.on("/favicon.ico", HTTP_GET, [this]()
              { server.send(204, "text/plain", ""); });
    registerCommandHandlers();
    server.onNotFound([this]()
                      { handleNotFound(); });
    server.begin();
}

void WebCommandInput::handleClient()
{
    server.handleClient();
}

void WebCommandInput::handleCommands()
{
    JsonDocument document;
    JsonArray commands = document["commands"].to<JsonArray>();

    uint8_t count = 0;
    const CommandDescriptor *descriptors = catalog.commands(count);
    for (uint8_t index = 0; index < count; index++)
    {
        JsonObject command = commands.add<JsonObject>();
        command["webName"] = descriptors[index].webName;
        command["serialKey"] = String(descriptors[index].serialKey);
        command["parameter"] = parameterName(descriptors[index].parameter);
        command["help"] = flashStringToString(descriptors[index].help);
    }

    String response;
    serializeJson(document, response);
    sendJson(200, response);
}

void WebCommandInput::handleStatus()
{
    JsonDocument document;
    document["wifi"] = WiFi.status() == WL_CONNECTED ? "connected" : "disconnected";
    document["ip"] = WiFi.localIP().toString();
    document["homingIdle"] = homing.isIdle();
    document["homingFaulted"] = homing.isFaulted();
    document["testActive"] = testController.isActive();

    JsonObject x = document["x"].to<JsonObject>();
    x["position"] = xMotor.position();
    x["range"] = xMotor.axisState().axisRangeSteps;

    JsonObject y = document["y"].to<JsonObject>();
    y["position"] = yMotor.position();
    y["range"] = yMotor.axisState().axisRangeSteps;

    JsonObject motion = document["motion"].to<JsonObject>();
    motion["microsteps"] = runtimeMotionSettings.microsteps;
    motion["speedHz"] = runtimeMotionSettings.speedHz;
    motion["acceleration"] = runtimeMotionSettings.acceleration;

    String response;
    serializeJson(document, response);
    sendJson(200, response);
}

void WebCommandInput::handleNotFound()
{
    if (server.method() == HTTP_OPTIONS)
    {
        sendCorsPreflight();
        return;
    }

    sendError(404, F("Not found."));
}

void WebCommandInput::handleCommandPost(const String &webName)
{
    const CommandDescriptor *descriptor = catalog.findByWebName(webName.c_str());
    if (descriptor == nullptr)
    {
        sendCommandResult({CommandStatus::UnknownCommand, F("Unknown command.")});
        return;
    }

    Command command = {};
    String errorMessage;
    if (!commandFromRequest(*descriptor, command, errorMessage))
    {
        JsonDocument document;
        document["status"] = statusName(CommandStatus::InvalidParameter);
        document["message"] = errorMessage;
        String response;
        serializeJson(document, response);
        sendJson(httpStatusCode(CommandStatus::InvalidParameter), response);
        return;
    }

    sendCommandResult(dispatcher.dispatch(command));
}

void WebCommandInput::registerCommandHandlers()
{
    uint8_t count = 0;
    const CommandDescriptor *descriptors = catalog.commands(count);
    for (uint8_t index = 0; index < count; index++)
    {
        const String webName = descriptors[index].webName;
        const String path = String("/api/commands/") + webName;
        server.on(path, HTTP_POST, [this, webName]()
                  { handleCommandPost(webName); });
        server.on(path, HTTP_OPTIONS, [this]()
                  { sendCorsPreflight(); });
    }
}

void WebCommandInput::sendCorsPreflight()
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.sendHeader("Access-Control-Max-Age", "600");
    server.send(204, "text/plain", "");
}

void WebCommandInput::sendJson(uint16_t statusCode, const String &response)
{
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(statusCode, "application/json", response);
}

bool WebCommandInput::commandFromRequest(const CommandDescriptor &descriptor, Command &command, String &errorMessage)
{
    command = {
        descriptor.group,
        descriptor.id,
        {descriptor.parameter, descriptor.parameterValue, 0, 0},
    };

    const String body = server.arg("plain");
    if (!isObjectBodyRequired(descriptor.parameter) && body.length() == 0)
    {
        return true;
    }

    JsonDocument document;
    const DeserializationError error = deserializeJson(document, body.length() == 0 ? "{}" : body);
    if (error)
    {
        errorMessage = "Malformed JSON body.";
        return false;
    }

    if (!document.is<JsonObject>())
    {
        errorMessage = "JSON body must be an object.";
        return false;
    }

    switch (descriptor.parameter)
    {
    case CommandParameterId::None:
    case CommandParameterId::Pattern:
        return true;
    case CommandParameterId::NormalizedPosition:
        if (!document["x"].is<uint16_t>() || !document["y"].is<uint16_t>())
        {
            errorMessage = "Expected numeric x and y fields.";
            return false;
        }
        command.parameter.xPosition = document["x"].as<uint16_t>();
        command.parameter.yPosition = document["y"].as<uint16_t>();
        return true;
    case CommandParameterId::Microsteps:
    case CommandParameterId::Speed:
    case CommandParameterId::Acceleration:
        if (!document["value"].is<uint32_t>())
        {
            errorMessage = "Expected numeric value field.";
            return false;
        }
        command.parameter.value = document["value"].as<uint32_t>();
        return true;
    }

    errorMessage = "Unsupported command parameter.";
    return false;
}

void WebCommandInput::sendCommandResult(const CommandResult &result)
{
    JsonDocument document;
    document["status"] = statusName(result.status);
    document["message"] = flashStringToString(result.message);

    String response;
    serializeJson(document, response);
    sendJson(httpStatusCode(result.status), response);
}

void WebCommandInput::sendError(uint16_t statusCode, const __FlashStringHelper *message)
{
    JsonDocument document;
    document["status"] = "error";
    document["message"] = flashStringToString(message);

    String response;
    serializeJson(document, response);
    sendJson(statusCode, response);
}