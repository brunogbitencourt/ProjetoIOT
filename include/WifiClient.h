#pragma once

#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"
#include "WiFi.h"
#include "WiFiClientSecure.h"

using namespace std;

class WifiClient{
        string ssid;
        string password;
        bool connected;
        bool reconnecting;
        SemaphoreHandle_t wifiSemaphore;
        WiFiClientSecure espClient; // Cria o objeto espClient


    public:
        WifiClient();
        ~WifiClient();
        void connect();
        void disconnect();        
        void reconnect();
        void loop(); 
};