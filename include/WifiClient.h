#pragma once

#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"
#include "WiFi.h"
#include "WiFiClientSecure.h"

using namespace std;

class WifiClient{
    private:
        string ssid;
        string password;
        bool connected;
        bool reconnecting;
        SemaphoreHandle_t wifiSemaphore;
        WiFiClientSecure espClient; // Cria o objeto espClient

        void configureNTP();


    public:
        WifiClient();
        ~WifiClient();
        void connectWiFi();
        void disconnect();        
        void reconnect();
        void loop(); 
        bool isConnected(); // Método para verificar o estado de conexão
};