#include "ActuatorManager.h"

ActuatorManager::ActuatorManager(MqttClient* mqttClient)
    : mqttClient(mqttClient)
{
    this->actuatorQueue = xQueueCreate(10, sizeof(char) * 512); // Adjust queue size as needed
}

ActuatorManager::~ActuatorManager()
{
    for (Actuator* actuator : this->actuators)
    {
        delete actuator;
    }
    vQueueDelete(this->actuatorQueue);
}

void ActuatorManager::addActuator(Actuator* actuator)
{
    this->actuators.push_back(actuator);
}

void ActuatorManager::updateActuatorPwmById(const char* id, int pwmValue)
{
    for (Actuator* actuator : actuators)
    {
        if (actuator->getId() == id) // Comparação direta entre std::string
        {
            actuator->setPwmOutput(pwmValue);
            char message[512];
            createActuatorJson(actuator, pwmValue, message, sizeof(message));
            addMessageToQueue(message);
            return;
        }
    }
    Serial.print("Actuator ID not found: ");
    Serial.println(id);
}


void ActuatorManager::sendToMqtt()
{
    char message[512]; // Adjust buffer size as needed
    while (getMessageFromQueue(message, sizeof(message)))
    {
        // Serial.print("Sending message to topic: ");
        // Serial.println(TOPIC_ACTUATORS);
        // Serial.println(message);
        this->mqttClient->publish(TOPIC_ACTUATORS, message);
    }
}

void ActuatorManager::createActuatorJson(Actuator* actuator, int pwmValue, char* buffer, size_t bufferSize)
{
    StaticJsonDocument<512> doc; // Adjust document size as needed
    char timeBuffer[25];

    // Get current time
    time_t now = time(nullptr);
    struct tm* p_tm = gmtime(&now);

    // Format time as UTC string
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             p_tm->tm_year + 1900, p_tm->tm_mon + 1, p_tm->tm_mday,
             p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);

    // Actuator information
    JsonObject root = doc.to<JsonObject>();
    root["Id"] = actuator->getId();
    root["Timestamp"] = timeBuffer;
    root["PwmOutput"] = pwmValue;

    size_t jsonSize = measureJson(doc);
    if (jsonSize > bufferSize)
    {
        Serial.println("JSON size exceeds buffer size");
        return;
    }

    serializeJson(doc, buffer, bufferSize);
}

void ActuatorManager::addMessageToQueue(const char* message)
{
    if (xQueueSend(this->actuatorQueue, message, portMAX_DELAY) != pdPASS)
    {
        Serial.println("Failed to add message to queue");
    }
}

bool ActuatorManager::getMessageFromQueue(char* messageBuffer, int bufferSize)
{
    if (xQueueReceive(this->actuatorQueue, messageBuffer, 0) == pdPASS)
    {
        return true;
    }
    else
    {
        Serial.println("Failed to retrieve message from queue");
        return false;
    }
}
