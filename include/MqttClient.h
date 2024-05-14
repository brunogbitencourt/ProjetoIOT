#pragma once

#include <Arduino.h>
#include <string>
#include "WifiClient.h"
#include <PubSubClient.h>

using namespace std;

class MqttClient {
private:
    WiFiClient espClient; // Objeto para conexão com o MQTT
    PubSubClient mqttClient; // Objeto para manipulação do cliente MQTT
    const char* broker;
    int port;
    const char* mqttId;

public:
    MqttClient(const char* broker, int port, const char* mqttId);
    ~MqttClient();
    void setup();
    void loop();
    void publish(const char* topic, const char* payload);
    void subscribe(const char* topic);
    bool isConnected();
    void callback(char* topic, byte* payload, unsigned int length);
    void reconnect();
};
