#include "SensorManager.h"

SensorManager::SensorManager(MqttClient* mqttClient)
    : mqttClient(mqttClient) {
    this->sensorQueue = xQueueCreate(10, sizeof(char) * 512); // Tamanho da fila e tamanho das mensagens
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

        char payload[512];
        this->createSensorJson(sensor, payload, sizeof(payload));
        this->addMessageToQueue(payload);
    }
}

void SensorManager::sendToMqtt() {
    char message[512];
    while (this->getMessageFromQueue(message, sizeof(message))) {
        this->mqttClient->publish("sensors/data", message);
    }
}

void SensorManager::addMessageToQueue(const char* message) {
    xQueueSend(this->sensorQueue, message, portMAX_DELAY);
}

bool SensorManager::getMessageFromQueue(char* messageBuffer, int bufferSize) {
    return xQueueReceive(this->sensorQueue, messageBuffer, 0) == pdPASS;
}

void SensorManager::createSensorJson(Sensor* sensor, char* buffer, size_t bufferSize) {
    StaticJsonDocument<512> doc;
    doc["id"] = sensor->getId();
    doc["description"] = sensor->getDescription();
    doc["type"] = sensor->getType();
    doc["outPutPin1"] = sensor->getOutPutPin1();
    doc["outPutPin2"] = sensor->getOutPutPin2();
    if (sensor->getType() == 0) {
        doc["analogValue"] = sensor->getAnalogValue();
    } else if (sensor->getType() == 1) {
        doc["digitalValue"] = sensor->getDigitalValue();
    }
    doc["timeStamp"] = sensor->getTimeStamp();

    serializeJson(doc, buffer, bufferSize);
}
