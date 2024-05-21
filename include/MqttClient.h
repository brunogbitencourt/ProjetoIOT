#pragma once

#include <Arduino.h>
#include <string>
#include "WifiClient.h"
#include <PubSubClient.h>
#include <freertos/queue.h>

#define MAX_PAYLOAD_LENGTH 256
#define QUEUE_LENGTH 10

using namespace std;

#ifndef MqttClient_h
#define MqttClient_h

class MqttClient {
private:
    WiFiClient espClient; // Objeto para conexão com o MQTT
    PubSubClient mqttClient; // Objeto para manipulação do cliente MQTT
    const char* broker;
    int port;
    const char* mqttId;

    QueueHandle_t mqttQueue;

    void callback(char* topic, byte* payload, unsigned int length);

public:
    MqttClient(const char* broker, int port, const char* mqttId);
    ~MqttClient();
    void setup();
    void loop();
    void publish(const char* topic, const char* payload);
    void subscribe(const char* topic);
    bool isConnected();
    bool connect(); // Adicionado
    void reconnect();
    int state(); // Adicionado
    bool receiveMessage(char* topic, char* payload);
};

struct MqttMessage {
    char topic[128];
    char payload[MAX_PAYLOAD_LENGTH];
};

#endif

