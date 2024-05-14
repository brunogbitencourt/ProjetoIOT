#include "MqttClient.h"

MqttClient::MqttClient(const char* broker, int port, const char* mqttId)
    : mqttClient(espClient), broker(broker), port(port), mqttId(mqttId) {}

MqttClient::~MqttClient() {}

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
    // Implemente o que deseja fazer com a mensagem recebida
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
            delay(5000);
        }
    }
}
