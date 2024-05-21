#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"
#include "Actuator.h"
#include "WiFiClient.h"
#include "MqttClient.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

using namespace std;

// Mutex handle
SemaphoreHandle_t mqttMutex;

//-----------------------------WiFi Parameters--------------------------------//
WifiClient wifiClient; 

//-----------------------------MQTT Parameters--------------------------------//
MqttClient mqttClient(BROKER_MQTT, BROKER_PORT, ID_MQTT);

void actuatorTask(void *pvParameters);
void wifiTask(void *pvParameters);
void mqttTask(void *pvParameters);
void mqttListenTask(void *pvParameters);

//-----------------------------Actuators--------------------------------//
Actuator pump1(1, "Pump 1 - Tank 1", 0, PUMP1_PIN, 0);
Actuator pump2(2, "Pump 2 - Tank 1", 0, PUMP2_PIN, 0);
Actuator pump3(3, "Pump 3 - Tank 2", 0, PUMP3_PIN, 0);
Actuator motor1(4, "Motor 1 - Tank 1", 1, MOTOR1_PIN, 0);
Actuator motor2(5, "Motor 2 - Tank 2", 1, MOTOR2_PIN, 0);
Actuator valve1(6, "Valve 1 - Tank 1", 2, VALVE1_PIN, 0);
Actuator valve2(7, "Valve 2 - Tank 2", 2, VALVE2_PIN, 0);

void setup() {
    Serial.begin(115200);

    // Inicializa o mutex
    mqttMutex = xSemaphoreCreateBinary();

    // Cria as tarefas FreeRTOS
    xTaskCreate(wifiTask, "Wifi Task", 2048, NULL, 1, NULL);
    xTaskCreate(mqttTask, "MQTT Task", 4096, NULL, 1, NULL);
    xTaskCreate(mqttListenTask, "MQTT Listen Task", 4096, NULL, 1, NULL);
    xTaskCreate(actuatorTask, "Pump 01 Task", 4096, NULL, 1, NULL);
    /*xTaskCreate(actuatorTask, "Pump 02 Task", 4096, &pump2, 1, NULL);
    xTaskCreate(actuatorTask, "Pump 03 Task", 4096, &pump3, 1, NULL);
    xTaskCreate(actuatorTask, "Motor 01 Task", 4096, &motor1, 1, NULL);
    xTaskCreate(actuatorTask, "Motor 02 Task", 4096, &motor2, 1, NULL);
    xTaskCreate(actuatorTask, "Valve 01 Task", 4096, &valve1, 1, NULL);
    xTaskCreate(actuatorTask, "Valve 02 Task", 4096, &valve2, 1, NULL);*/
}

void loop() {
    // Não faz nada no loop principal, já que tudo é gerenciado pelas tarefas FreeRTOS
}

void actuatorTask(void *pvParameters) {
    while(1) { 
        if (Serial.available() > 0) {      
            String arduinoString  = Serial.readString();
            string valSerial = string(arduinoString.c_str());

            if(valSerial == "a1l"){
                Serial.println("Pump 1 - Turn On");
                pump1.setPwmOutput(255);
            }      
            else if (valSerial == "a1d"){
                Serial.println("Pump 1 - Turn Off");
                pump1.setPwmOutput(0);        
            }
            else if (valSerial == "a2l"){
                Serial.println("Pump 2 - Turn On");
                pump2.setPwmOutput(255);
            }      
            else if (valSerial == "a2d"){
                Serial.println("Pump 2 - Turn Off");
                pump2.setPwmOutput(0);        
            }
            else if (valSerial == "a3l"){
                Serial.println("Pump 3 - Turn On");
                pump3.setPwmOutput(255);
            }      
            else if (valSerial == "a3d"){
                Serial.println("Pump 3 - Turn Off");
                pump3.setPwmOutput(0);        
            }
            else if (valSerial == "a4l"){
                Serial.println("Motor 1 - Turn On");
                motor1.setPwmOutput(255);
            }      
            else if (valSerial == "a4d"){
                Serial.println("Motor 1 - Turn Off");
                motor1.setPwmOutput(0);        
            }
            else if (valSerial == "a5l"){
                Serial.println("Motor 2 - Turn On");
                motor2.setPwmOutput(255);
            }      
            else if (valSerial == "a5d"){
                Serial.println("Motor 2 - Turn Off");
                motor2.setPwmOutput(0);        
            }
            else if (valSerial == "a6l"){
                Serial.println("Valve 1 - Turn On");
                valve1.setPwmOutput(255);
            }      
            else if (valSerial == "a6d"){
                Serial.println("Valve 1 - Turn Off");
                valve1.setPwmOutput(0);        
            }
            else if (valSerial == "a7l"){
                Serial.println("Valve 2 - Turn On");
                valve2.setPwmOutput(255);
            }      
            else if (valSerial == "a7d"){
                Serial.println("Valve 2 - Turn Off");
                valve2.setPwmOutput(0);
            }
            else{
                Serial.println("Invalid Command");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Espera 200 ms        
    }
}

void wifiTask(void *pvParameters) {
    int counter = 1;
    while (1) {
        Serial.print("Attempt: ");
        Serial.println(counter);
        wifiClient.connectWiFi();
        while (wifiClient.isConnected()) {
            wifiClient.loop();
            vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos
        }
        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo antes de tentar reconectar
    }
}

void mqttTask(void *pvParameters) {
    while (1) {
        if (wifiClient.isConnected()) { // Verifica se o WiFi está conectado
            if (!mqttClient.isConnected()) { // Verifica se o MQTT está conectado
                mqttClient.setup();
                Serial.println("Attempting MQTT connection...");
                if (mqttClient.connect()) { // Tenta conectar
                    Serial.println("connected");
                    xSemaphoreGive(mqttMutex); // Libera o mutex se conectado
                } else {
                    Serial.print("failed, rc=");
                    Serial.print(mqttClient.state());
                    Serial.println(" try again in 5 seconds");
                    vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos antes de tentar novamente
                }
            } else {
                mqttClient.loop(); // Mantém a conexão MQTT ativa e processa mensagens
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Ajuste o atraso conforme necessário
    }
}

void mqttListenTask(void *pvParameters) {
    const char* topic = "sistema/sensor01";
    bool isSubscribed = false;

    while (1) {
        if (mqttClient.isConnected()) {
            if (!isSubscribed) {
                mqttClient.subscribe(topic); // Inscreve-se no tópico desejado
                isSubscribed = true;
            }
            mqttClient.loop(); // Processa mensagens MQTT
        } else {
            isSubscribed = false; // Reinicia a inscrição se a conexão for perdida
            Serial.println("Waiting for MQTT connection...");
            xSemaphoreTake(mqttMutex, portMAX_DELAY); // Aguarda até que a conexão esteja ativa
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
