#include "SensorManager.h"
#include <ctime>
#include <ArduinoJson.h>
#include "GlobalConfig.h"

SensorManager::SensorManager(MqttClient *mqttClient)
    : mqttClient(mqttClient)
{
    this->sensorQueue = xQueueCreate(10, sizeof(char) * 2048); // Ajuste o tamanho da fila e das mensagens conforme necessário
}

SensorManager::~SensorManager()
{
    for (Sensor *sensor : this->sensors)
    {
        delete sensor;
    }
    vQueueDelete(this->sensorQueue);
}

void SensorManager::addSensor(Sensor *sensor)
{
    this->sensors.push_back(sensor);
}

void SensorManager::readSensors()
{
    for (Sensor *sensor : this->sensors)
    {
        if (sensor->getType() == 0)
        {
            sensor->setAnalogValue(sensor->getAnalogValue());
        }
        else if (sensor->getType() == 1)
        {
            sensor->setDigitalValue(sensor->getDigitalValue());
        }
        sensor->updateTimeStamp();

        if(mqttClient->isConnected()){
            char payload[2048]; // Ajuste o tamanho do buffer aqui
            this->createSensorJson(sensor, payload, sizeof(payload));
            this->mqttClient->publish(TOPIC_SENSORS, payload);
        }
    }
}

void SensorManager::sendToMqtt()
{
    char message[2048]; // Ajuste o tamanho do buffer aqui
    while (this->getMessageFromQueue(message, sizeof(message)))
    {
        // Serial.print("Sending message to topic: ");
        // Serial.println(TOPIC_SENSORS);
        // Serial.println(message);
        
    }
}

void SensorManager::addMessageToQueue(const char *message)
{
    if (xQueueSend(this->sensorQueue, message, portMAX_DELAY) != pdPASS)
    {
        Serial.println("Failed to add message to queue");
    }
}

bool SensorManager::getMessageFromQueue(char *messageBuffer, int bufferSize)
{
    if (xQueueReceive(this->sensorQueue, messageBuffer, 0) == pdPASS)
    {
        return true;
    }
    else
    {
        Serial.println("Failed to retrieve message from queue");
        return false;
    }
}

void SensorManager::createSensorJson(Sensor *sensor, char *buffer, size_t bufferSize)
{
    StaticJsonDocument<2048> doc; // Ajuste o tamanho do documento se necessário
    char timeBuffer[25];

    // Obtém o tempo atual
    time_t now = time(nullptr);

    // Verifica se o tempo atual é válido (ou seja, se foi atualizado)
    if (now <= 0)
    {
        Serial.println("Time not updated. JSON not created.");
        return;
    }

    struct tm *p_tm = gmtime(&now);

    // Formata o tempo como string UTC
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             p_tm->tm_year + 1900, p_tm->tm_mon + 1, p_tm->tm_mday,
             p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);

    // Sensor information
    JsonObject root = doc.to<JsonObject>();
    root["Id"] = sensor->getId();
    root["Timestamp"] = timeBuffer;

    if (sensor->getType() == 1)
    { // Analog sensor
        root["AnalogValue"] = sensor->getAnalogValue();
        root["Unit"] = "analog unit"; // Substitua pela unidade correta
    }
    else if (sensor->getType() == 2)
    { // Digital sensor
        root["DigitalValue"] = sensor->getDigitalValue();
    }

    size_t jsonSize = measureJson(doc);
    if (jsonSize > bufferSize)
    {
        Serial.println("JSON size exceeds buffer size");
        return;
    }

    serializeJson(doc, buffer, bufferSize);
}


const std::vector<Sensor*>& SensorManager::getSensors() const
{
    return sensors;
}