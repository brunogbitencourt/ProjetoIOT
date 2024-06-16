#pragma once

#include <vector>
#include "Sensor.h"
#include "MqttClient.h"
#include <ArduinoJson.h>

class SensorManager {
public:
    SensorManager(MqttClient* mqttClient);
    ~SensorManager();

    void addSensor(Sensor* sensor);
    void readSensors();
    void sendToMqtt();
    void addMessageToQueue(const char* message);
    bool getMessageFromQueue(char* messageBuffer, int bufferSize);

    const std::vector<Sensor*>& getSensors() const; // Declaration of getSensors method

private:
    std::vector<Sensor*> sensors;
    MqttClient* mqttClient;
    QueueHandle_t sensorQueue;

    void createSensorJson(Sensor* sensor, char* buffer, size_t bufferSize);
};
