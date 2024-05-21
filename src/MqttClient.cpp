#include "MqttClient.h"

MqttClient::MqttClient(const char* broker, int port, const char* mqttId)
    : mqttClient(espClient), broker(broker), port(port), mqttId(mqttId) {
        mqttQueue = xQueueCreate(QUEUE_LENGTH, sizeof(MqttMessage));
    }

MqttClient::~MqttClient() {
    if (mqttQueue != NULL) {
        vQueueDelete(mqttQueue);
    }
}

void MqttClient::setup() {
    mqttClient.setServer(broker, port);
    mqttClient.setCallback([&](char* topic, byte* payload, unsigned int length) {
        this->callback(topic, payload, length);
    });
    reconnect();
}

void MqttClient::loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
}

void MqttClient::publish(const char* topic, const char* payload) {
    if (mqttClient.connected()) {
        mqttClient.publish(topic, payload);
    }
}

void MqttClient::subscribe(const char* topic) {
    if (mqttClient.connected()) {
        mqttClient.subscribe(topic);
    }
}

bool MqttClient::isConnected() {
    return mqttClient.connected();
}

void MqttClient::callback(char* topic, byte* payload, unsigned int length) {
    if (length < MAX_PAYLOAD_LENGTH) {
        MqttMessage message;
        strncpy(message.topic, topic, sizeof(message.topic));
        strncpy(message.payload, (char*)payload, length);
        message.payload[length] = '\0';

        if (xQueueSend(mqttQueue, &message, portMAX_DELAY) != pdPASS) {
            Serial.println("Failed to enqueue MQTT message");
        }
    } else {
        Serial.println("Received payload is too large");
    }
}

bool MqttClient::receiveMessage(char* topic, char* payload) {
    MqttMessage message;
    if (xQueueReceive(mqttQueue, &message, 0) == pdPASS) {
        strncpy(topic, message.topic, sizeof(message.topic));
        strncpy(payload, message.payload, MAX_PAYLOAD_LENGTH);
        return true;
    }
    return false;
}

bool MqttClient::connect() {
    return mqttClient.connect(mqttId);
}

void MqttClient::reconnect() {
    while (!mqttClient.connected()) {
        Serial.println("Attempting MQTT connection...");
        if (mqttClient.connect(mqttId)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

int MqttClient::state() {
    return mqttClient.state();
}
