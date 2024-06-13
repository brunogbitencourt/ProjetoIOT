#include "SensorManager.h"
#include <ctime>
#include <ArduinoJson.h>

SensorManager::SensorManager(MqttClient* mqttClient)
    : mqttClient(mqttClient) {
    this->sensorQueue = xQueueCreate(10, sizeof(char) * 2048); // Tamanho da fila e tamanho das mensagens
}

SensorManager::~SensorManager() {
    for (Sensor* sensor : this->sensors) {
        delete sensor;
    }
    vQueueDelete(this->sensorQueue);
}

void SensorManager::addSensor(Sensor* sensor) {
    this->sensors.push_back(sensor);
}

void SensorManager::readSensors() {
    for (Sensor* sensor : this->sensors) {
        if (sensor->getType() == 0) {
            sensor->setAnalogValue(sensor->getAnalogValue());
        } else if (sensor->getType() == 1) {
            sensor->setDigitalValue(sensor->getDigitalValue());
        }
        sensor->updateTimeStamp();

        char payload[2048];
        this->createSensorJson(sensor, payload, sizeof(payload));
        this->addMessageToQueue(payload);
    }
}

void SensorManager::sendToMqtt() {
    char message[2048];
    while (this->getMessageFromQueue(message, sizeof(message))) {
        Serial.print("Sendding message to topic: ");
        Serial.println("sensors/data");
        Serial.println(message);
        this->mqttClient->publish("sensors/data", message);
    }
}

void SensorManager::addMessageToQueue(const char* message) {
    // Serial.println("Adding message to queue:");
    // Serial.println(message);
    if (xQueueSend(this->sensorQueue, message, portMAX_DELAY) != pdPASS) {
        Serial.println("Failed to add message to queue");
    }
}

bool SensorManager::getMessageFromQueue(char* messageBuffer, int bufferSize) {
    if (xQueueReceive(this->sensorQueue, messageBuffer, 0) == pdPASS) {
        // Serial.println("Message retrieved from queue");
        return true;
    } else {
        Serial.println("Failed to retrieve message from queue");
        return false;
    }
}

void SensorManager::createSensorJson(Sensor* sensor, char* buffer, size_t bufferSize) {
    StaticJsonDocument<2048> doc;
    char timeBuffer[25];

    // Obtém o tempo atual
    time_t now = time(nullptr);
    struct tm* p_tm = gmtime(&now);

    // Formata o tempo como string UTC
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             p_tm->tm_year + 1900, p_tm->tm_mon + 1, p_tm->tm_mday,
             p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);

    JsonObject root = doc.createNestedObject("devices");
    JsonObject sensors = root.createNestedObject("sensors");
    JsonObject sensorData = root.createNestedObject("sensors_data");

    // Sensor information
    JsonObject sensorObj = sensors.createNestedObject(sensor->getId());
    sensorObj["Description"] = sensor->getDescription();
    sensorObj["Id"] = sensor->getId();
    sensorObj["OutputPin1"] = sensor->getOutPutPin1();
    sensorObj["OutputPin2"] = sensor->getOutPutPin2();
    sensorObj["Type"] = sensor->getType();

    // Sensor data
    JsonObject sensorDataObj = sensorData.createNestedObject(sensor->getId()).createNestedObject(timeBuffer);
    sensorDataObj["Id"] = sensor->getId();
    sensorDataObj["Timestamp"] = timeBuffer;
    if (sensor->getType() == 0) {
        sensorDataObj["AnalogValue"] = sensor->getAnalogValue();
        sensorDataObj["Unit"] = "analog unit"; // substitua pela unidade correta
    } else if (sensor->getType() == 1) {
        sensorDataObj["DigitalValue"] = sensor->getDigitalValue();
        sensorDataObj["Unit"] = "digital unit"; // substitua pela unidade correta
    }

    serializeJson(doc, buffer, bufferSize);
}