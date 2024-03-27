#include <Arduino.h>
//#include <FreeRTOS.h>
#include "Pump.h"
#include "USSensor.h"

// Defina os pinos de saída e de controle PWM
#define OUTPUT_PIN 2

#define TRIG_PIN_S1 4
#define ECHO_PIN_S1 5

void pumpTask(void *pvParameters);

void usTask(void *pvParameters);

USSensor us_s1(ECHO_PIN_S1, TRIG_PIN_S1);

Pump pump(OUTPUT_PIN, 0);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);


  xTaskCreate(pumpTask, "Pump Task", 1024, NULL, 1, NULL);
  
  xTaskCreate(usTask, "Uss Task", 1024, NULL, 1, NULL);

  // vTaskStartScheduler();


}

void loop() {
  // put your main code here, to run repeatedly:
}




void usTask(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares
        Serial.println("Uss Task");
        Serial.print("Distance cm: ");
        Serial.println(us_s1.getDistance());

        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 1 segundo
    }
}

void pumpTask(void *pvParameters) {
    (void)pvParameters;

    while(1) {
        // Lógica da tarefa da bomba aqui
        // Por exemplo, ligue e desligue a bomba em intervalos regulares
        Serial.println("Pump Task");
        Serial.println("Ligando a bomba");
        pump.turnOn(255); // Ligar a bomba com saída PWM de 255 (100%)
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 1 segundo
        pump.turnOff(); // Desligar a bomba
        Serial.println("Desligando a bomba");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 segundos
    }
}