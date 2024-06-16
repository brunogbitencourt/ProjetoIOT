#pragma once

#include <vector>
#include "Actuator.h"
#include "MqttClient.h"
#include <ArduinoJson.h>

class ActuatorManager {
public:
    ActuatorManager(MqttClient* mqttClient);
    ~ActuatorManager();

    void addActuator(Actuator* actuator);
    void updateActuatorPwmById(const char* id, int pwmValue);
    void sendToMqtt();

private:
    std::vector<Actuator*> actuators;
    MqttClient* mqttClient;
    QueueHandle_t actuatorQueue;

    void createActuatorJson(Actuator* actuator, int pwmValue, char* buffer, size_t bufferSize);
    void addMessageToQueue(const char* message);
    bool getMessageFromQueue(char* messageBuffer, int bufferSize);
};
