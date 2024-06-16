#include <Arduino.h>
#include <string>
#include "GlobalConfig.h"
#include "Actuator.h"
#include "WiFiClient.h"
#include "MqttClient.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "SensorManager.h"
#include "ActuatorManager.h"

using namespace std;

// Mutex handle
SemaphoreHandle_t mqttMutex;

//-----------------------------WiFi Parameters--------------------------------//
WifiClient wifiClient; 

//-----------------------------MQTT Parameters--------------------------------//
MqttClient mqttClient(BROKER_MQTT, BROKER_PORT, ID_MQTT);

SensorManager sensorManager(&mqttClient);
ActuatorManager actuatorManager(&mqttClient);

void actuatorTask(void *pvParameters);
void wifiTask(void *pvParameters);
void mqttTask(void *pvParameters);
void mqttListenTask(void *pvParameters);
void mqttPublishTask(void *pvParameters);
void sensorTask(void *pvParameters);
void sensorToMqttTask(void *pvParameters);
void actuatorToMqttTask(void *pvParameters);
void setActuatorPwmById(const String& id, int pwmValue);
void analyzeSensorData(Sensor* sensor);

//-----------------------------Actuators--------------------------------//
Actuator pump1("AP_01", "Pump 1 - Tank 1", 0, PUMP1_PIN, 0);
Actuator pump2("AP_02", "Pump 2 - Tank 1", 0, PUMP2_PIN, 0);
Actuator pump3("AP_03", "Pump 3 - Tank 2", 0, PUMP3_PIN, 0);
Actuator motor1("AM_01", "Motor 1 - Tank 1", 1, MOTOR1_PIN, 0);
Actuator motor2("AM_02", "Motor 2 - Tank 2", 1, MOTOR2_PIN, 0);
Actuator valve1("AV_01", "Valve 1 - Tank 1", 2, VALVE1_PIN, 0);
Actuator valve2("AV_02", "Valve 2 - Tank 2", 2, VALVE2_PIN, 0);

void setup() {
    Serial.begin(115200);

    // Adiciona sensores ao gerenciador
    sensorManager.addSensor(new Sensor("uss1", "Sensor Ultrassonico 1", 1, SENSOR_A1_EPIN, SENSOR_A1_TPIN));
    sensorManager.addSensor(new Sensor("uss2", "Sensor Ultrassonico 2", 1, SENSOR_A2_EPIN, SENSOR_A2_TPIN));
    sensorManager.addSensor(new Sensor("ds1", "Sensor Digital 1", 2, SENSOR_D1_PIN, NULL));
    sensorManager.addSensor(new Sensor("ds2", "Sensor Digital 2", 2, SENSOR_D2_PIN, NULL));

    // Adiciona atuadores ao gerenciador
    actuatorManager.addActuator(&pump1);
    actuatorManager.addActuator(&pump2);
    actuatorManager.addActuator(&pump3);
    actuatorManager.addActuator(&motor1);
    actuatorManager.addActuator(&motor2);
    actuatorManager.addActuator(&valve1);
    actuatorManager.addActuator(&valve2);

    // Inicializa o mutex
    mqttMutex = xSemaphoreCreateBinary();

    // Cria as tarefas FreeRTOS
    xTaskCreate(wifiTask, "Wifi Task", 2048, NULL, 1, NULL);
    xTaskCreate(mqttTask, "MQTT Task", 4096, NULL, 1, NULL);
    xTaskCreate(mqttListenTask, "MQTT Listen Task", 4096, NULL, 1, NULL);
    xTaskCreate(actuatorTask, "Pump 01 Task", 4096, NULL, 1, NULL);
    xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 1, NULL);
    xTaskCreate(sensorToMqttTask, "Sensor to MQTT Task", 4096, NULL, 1, NULL);
    xTaskCreate(actuatorToMqttTask, "Actuator to MQTT Task", 4096, NULL, 1, NULL);
    // xTaskCreate(mqttPublishTask, "MQTT Publish Task", 4096, NULL, 1, NULL);
}

void loop() {
    // Não faz nada no loop principal, já que tudo é gerenciado pelas tarefas FreeRTOS
}

void actuatorTask(void *pvParameters) {
    char topic[128];
    char payload[MAX_PAYLOAD_LENGTH];

    while(1) { 
        if (mqttClient.receiveMessage(topic, payload)) {   
            // payload = "AP_XX-YY", onde "AP_XX" é o ID do atuador e "YY" é o valor em porcentagem para definir o PWM.   
            String valSerial = String(payload);
            int hyphenIndex = valSerial.indexOf('-');
            if (hyphenIndex != -1) {
                String id = valSerial.substring(0, hyphenIndex);
                int percentage = valSerial.substring(hyphenIndex + 1).toInt();
                int pwmValue = map(percentage, 0, 100, 0, 255);

                setActuatorPwmById(id, pwmValue);
            } else {
                Serial.println("Invalid command format: " + valSerial);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Espera 200 ms        
    }
}

void setActuatorPwmById(const String& id, int pwmValue) {
    actuatorManager.updateActuatorPwmById(id.c_str(), pwmValue);
    if (id == "AP_01") {
        Serial.println("Setting Pump 1 PWM to " + String(pwmValue));
        pump1.setPwmOutput(pwmValue);
    } else if (id == "AP_02") {
        Serial.println("Setting Pump 2 PWM to " + String(pwmValue));
        pump2.setPwmOutput(pwmValue);
    } else if (id == "AP_03") {
        Serial.println("Setting Pump 3 PWM to " + String(pwmValue));
        pump3.setPwmOutput(pwmValue);
    } else if (id == "AM_01") {
        Serial.println("Setting Motor 1 PWM to " + String(pwmValue));
        motor1.setPwmOutput(pwmValue);
    } else if (id == "AM_02") {
        Serial.println("Setting Motor 2 PWM to " + String(pwmValue));
        motor2.setPwmOutput(pwmValue);
    } else if (id == "AV_01") {
        Serial.println("Setting Valve 1 PWM to " + String(pwmValue));
        valve1.setPwmOutput(pwmValue);
    } else if (id == "AV_02") {
        Serial.println("Setting Valve 2 PWM to " + String(pwmValue));
        valve2.setPwmOutput(pwmValue);
    } else {
        Serial.println("Unknown actuator ID: " + id);
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
    const char* topic = TOPIC_ACTIONS;
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
            // Serial.println("Waiting for MQTT connection...");
            xSemaphoreTake(mqttMutex, portMAX_DELAY); // Aguarda até que a conexão esteja ativa
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void mqttPublishTask(void *pvParameters) {
    const char* topic = "sistema/actuator01";
    const char* payload = "Hello from ESP32";

    while (1) {
        if (mqttClient.isConnected()) {
            mqttClient.publish(topic, payload); // Envia a mensagem para o tópico
            Serial.println("Message published");
        } else {
            Serial.println("MQTT not connected, cannot publish");
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Ajuste o intervalo de publicação conforme necessário
    }
}

void sensorTask(void *pvParameters) {
    while (1) {
        sensorManager.readSensors();

        // Analisar os dados dos sensores após a leitura
        for (Sensor* sensor : sensorManager.getSensors()) {
            analyzeSensorData(sensor);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sensorToMqttTask(void *pvParameters) {
    while (1) {
        sensorManager.sendToMqtt();
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}

void actuatorToMqttTask(void *pvParameters) {
    while (1) {
        actuatorManager.sendToMqtt();
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}

void analyzeSensorData(Sensor* sensor) {
    if (sensor->getType() == 1) { // Analog sensor
        int analogValue = sensor->getAnalogValue();

        if (sensor->getId() == "uss1") {
            if (analogValue < 100) {
                setActuatorPwmById("AV_01", 255); // Valve 1 fully open
            } else if (analogValue < 500) {
                setActuatorPwmById("AV_01", 128); // Valve 1 partially open
            } else {
                setActuatorPwmById("AV_01", 0); // Valve 1 closed
            }
        } else if (sensor->getId() == "uss2") {
            if (analogValue < 100) {
                setActuatorPwmById("AV_02", 255); // Valve 2 fully open
            } else if (analogValue < 500) {
                setActuatorPwmById("AV_02", 128); // Valve 2 partially open
            } else {
                setActuatorPwmById("AV_02", 0); // Valve 2 closed
            }
        }
    } else if (sensor->getType() == 2) { // Digital sensor
        bool digitalValue = sensor->getDigitalValue();

        if (sensor->getId() == "ds1") {
            if (digitalValue) {
                setActuatorPwmById("AM_01", 255); // Motor 1 on
            } else {
                setActuatorPwmById("AM_01", 0); // Motor 1 off
            }
        } else if (sensor->getId() == "ds2") {
            if (digitalValue) {
                setActuatorPwmById("AM_02", 255); // Motor 2 on
            } else {
                setActuatorPwmById("AM_02", 0); // Motor 2 off
            }
        }
    }
}