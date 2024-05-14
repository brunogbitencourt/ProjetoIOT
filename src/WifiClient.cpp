#include "GlobalConfig.h"
#include "WifiClient.h"


WifiClient::WifiClient()
{
    this->ssid = WIFI_SSID;
    this->password = WIFI_PASS;
    this->connected = false;
    this->reconnecting = false;
    this->wifiSemaphore = xSemaphoreCreateMutex();  

}

WifiClient::~WifiClient()
{    
    //delete this->espClient;
}

void WifiClient::connectWiFi()
{
    if (xSemaphoreTake(this->wifiSemaphore, portMAX_DELAY) == pdTRUE)
    {
        try
        {
            if (WiFi.status() != WL_CONNECTED)
            {
                Serial.print("Connecting to ");
                Serial.print(this->ssid.c_str());

                WiFi.begin(this->ssid.c_str(), this->password.c_str());
                int retries = 0;
                while (WiFi.status() != WL_CONNECTED && retries < 10)
                {
                    Serial.print(".");
                    vTaskDelay(pdMS_TO_TICKS(500));
                    retries++;
                }
                if (WiFi.status() == WL_CONNECTED)
                {
                    this->connected = true;
                    this->reconnecting = false;
                    Serial.println("");
                    Serial.println("WiFi connected!");
                }
                else
                {
                    this->connected = false;
                    this->reconnecting = true;
                    Serial.println("WiFi not connected!");
                }
            }
            xSemaphoreGive(this->wifiSemaphore);
        }
        catch(const std::exception &e)
        {
            Serial.print("Caught an exception while connecting to WiFi: ");
            Serial.println(e.what());
        }
        
    }
}

void WifiClient::disconnect()
{
    if (xSemaphoreTake(this->wifiSemaphore, portMAX_DELAY) == pdTRUE)
    {
        WiFi.disconnect();
        this->connected = false;
        this->reconnecting = false;
        xSemaphoreGive(this->wifiSemaphore);
    }
}

void WifiClient::reconnect()
{
    if (xSemaphoreTake(this->wifiSemaphore, portMAX_DELAY) == pdTRUE)
    {
        this->disconnect();
        this->connectWiFi();
        xSemaphoreGive(this->wifiSemaphore);
    }
}


void WifiClient::loop()
{
    if (xSemaphoreTake(this->wifiSemaphore, portMAX_DELAY) == pdTRUE)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            this->reconnect();
        }
        xSemaphoreGive(this->wifiSemaphore);
    }
}

bool WifiClient::isConnected() // Implementação do método isConnected()
{
    return this->connected;
}